/**
 * ARCS VR Control Application
 * WebXR controller tracking and robot control via WebSocket
 * Adapted from XLeVR implementation
 */

// Global state
let vrConnected = false;
let robotConnected = false;
let videoInitialized = false;

// A-Frame controller-updater component
AFRAME.registerComponent('controller-updater', {
    init: function () {
        console.log('VR Controller updater initialized');

        // Get controller entities
        this.leftHand = document.querySelector('#leftHand');
        this.rightHand = document.querySelector('#rightHand');
        this.leftHandInfoText = document.querySelector('#leftHandInfo');
        this.rightHandInfoText = document.querySelector('#rightHandInfo');
        this.headset = document.querySelector('#headset');
        this.statusText = document.querySelector('#statusText');
        this.armModeText = document.querySelector('#armModeText');
        this.gripperText = document.querySelector('#gripperText');

        // Controller state
        this.leftGripDown = false;
        this.rightGripDown = false;
        this.leftTriggerDown = false;
        this.rightTriggerDown = false;

        // Relative tracking for arm control
        this.rightGripInitialPosition = null;
        this.rightGripInitialQuaternion = null;
        this.armBasePosition = { shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 0, wrist_flex: 0, wrist_roll: 0 };

        // Movement deadzone
        this.thumbstickDeadzone = 0.15;

        // Arm sensitivity scaling (meters to degrees)
        this.armScale = {
            pan: 90,      // shoulder pan degrees per meter X
            lift: 90,     // shoulder lift degrees per meter Y
            reach: 90,    // elbow flex degrees per meter Z
            wristRoll: 45 // wrist roll degrees per 90° controller rotation
        };

        // Throttling state
        this.lastArmUpdate = 0;
        this.armUpdateInterval = 100; // 10Hz limit
        this.lastMoveUpdate = 0;
        this.moveUpdateInterval = 100; // 10Hz limit
        this.isMoving = false; // To track if movement command was last sent

        // Force hide A-Frame's default VR button
        const style = document.createElement('style');
        style.innerHTML = '.a-enter-vr-button { display: none !important; }';
        document.head.appendChild(style);

        // WebSocket setup
        this.websocket = null;
        this.setupWebSocket();

        // Setup video feed
        this.setupVideoFeed();

        // Controller event listeners
        this.setupControllerEvents();

        // Update desktop status indicators
        this.updateDesktopStatus();
        setInterval(() => this.updateDesktopStatus(), 2000);
    },

    setupWebSocket: function () {
        const serverHostname = window.location.hostname;
        const websocketPort = 8442;
        const websocketUrl = `ws://${serverHostname}:${websocketPort}`;

        console.log(`Connecting to WebSocket: ${websocketUrl}`);

        try {
            this.websocket = new WebSocket(websocketUrl);

            this.websocket.onopen = () => {
                console.log('WebSocket connected');
                this.updateStatus('Connected', '#4CAF50');
                document.querySelector('#wsStatus')?.classList.add('connected');
            };

            this.websocket.onerror = (event) => {
                console.error('WebSocket error:', event);
                this.updateStatus('Connection Error', '#f44336');
            };

            this.websocket.onclose = (event) => {
                console.log('WebSocket closed:', event.code, event.reason);
                this.updateStatus('Disconnected', '#ff9800');
                document.querySelector('#wsStatus')?.classList.remove('connected');
                this.websocket = null;

                // Attempt reconnection after 3 seconds
                setTimeout(() => this.setupWebSocket(), 3000);
            };

            this.websocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.status) {
                        robotConnected = data.status === 'ok';
                    }
                } catch (e) {
                    console.log('Server message:', event.data);
                }
            };
        } catch (error) {
            console.error('WebSocket connection failed:', error);
            this.updateStatus('Connection Failed', '#f44336');
        }
    },

    setupVideoFeed: function () {
        const videoPanel = document.querySelector('#videoPanel');
        if (!videoPanel) return;

        // Use MJPEG stream as image source
        const img = document.createElement('img');
        img.id = 'cameraImg';
        img.crossOrigin = 'anonymous';
        img.src = '/video_feed';
        img.style.display = 'none';
        document.body.appendChild(img);

        // Create canvas to convert MJPEG to texture
        const canvas = document.createElement('canvas');
        canvas.id = 'videoCanvas';
        canvas.width = 640;
        canvas.height = 480;
        canvas.style.display = 'none';
        document.body.appendChild(canvas);

        const ctx = canvas.getContext('2d');

        // Set canvas as texture immediately
        videoPanel.setAttribute('material', {
            src: '#videoCanvas',
            shader: 'flat'
        });

        // Continuous update loop for video feed
        const updateCanvas = () => {
            if (img.complete && img.naturalWidth > 0) {
                canvas.width = img.naturalWidth;
                canvas.height = img.naturalHeight;
                ctx.drawImage(img, 0, 0);

                // Force material update by touching the src attribute
                const material = videoPanel.getObject3D('mesh')?.material;
                if (material && material.map) {
                    material.map.needsUpdate = true;
                }
            }
            requestAnimationFrame(updateCanvas);
        };

        // Start update loop immediately
        updateCanvas();
    },

    setupControllerEvents: function () {
        if (!this.leftHand || !this.rightHand) {
            console.error('Controller entities not found');
            return;
        }

        // Left controller events
        this.leftHand.addEventListener('gripdown', () => {
            this.leftGripDown = true;
            console.log('Left grip down');
        });

        this.leftHand.addEventListener('gripup', () => {
            this.leftGripDown = false;
            console.log('Left grip up');
        });

        this.leftHand.addEventListener('triggerdown', () => {
            this.leftTriggerDown = true;
        });

        this.leftHand.addEventListener('triggerup', () => {
            this.leftTriggerDown = false;
        });

        // Right controller events (arm control)
        this.rightHand.addEventListener('gripdown', () => {
            this.rightGripDown = true;
            console.log('Right grip down - arm tracking enabled');

            // Store initial position for relative tracking
            if (this.rightHand.object3D) {
                this.rightGripInitialPosition = this.rightHand.object3D.position.clone();
                this.rightGripInitialQuaternion = this.rightHand.object3D.quaternion.clone();

                // Fetch current arm position as base
                this.fetchArmPosition();
            }

            if (this.armModeText) {
                this.armModeText.setAttribute('value', '🦾 ARM TRACKING');
            }
        });

        this.rightHand.addEventListener('gripup', () => {
            this.rightGripDown = false;
            this.rightGripInitialPosition = null;
            this.rightGripInitialQuaternion = null;
            console.log('Right grip up - arm tracking disabled');

            if (this.armModeText) {
                this.armModeText.setAttribute('value', '');
            }

            // Send release message
            this.sendGripRelease('right');
        });

        this.rightHand.addEventListener('triggerdown', () => {
            this.rightTriggerDown = true;
            console.log('Right trigger down - gripper close');
            this.sendGripperCommand(true);

            if (this.gripperText) {
                this.gripperText.setAttribute('value', '✊ GRIPPER CLOSED');
            }
        });

        this.rightHand.addEventListener('triggerup', () => {
            this.rightTriggerDown = false;
            console.log('Right trigger up - gripper open');
            this.sendGripperCommand(false);

            if (this.gripperText) {
                this.gripperText.setAttribute('value', '');
            }
        });
    },

    fetchArmPosition: function () {
        console.log('[VR-ARM] Fetching initial arm position...');
        fetch('/arm_position')
            .then(res => res.json())
            .then(data => {
                console.log('[VR-ARM] Got arm position:', data);
                if (data.error) {
                    console.error('[VR-ARM] Arm error:', data.error);
                    return;
                }
                if (data.positions) {
                    this.armBasePosition = {
                        shoulder_pan: data.positions.shoulder_pan || 0,
                        shoulder_lift: data.positions.shoulder_lift || 0,
                        elbow_flex: data.positions.elbow_flex || 0,
                        wrist_flex: data.positions.wrist_flex || 0,
                        wrist_roll: data.positions.wrist_roll || 0
                    };
                    console.log('[VR-ARM] Base position set:', this.armBasePosition);
                }
            })
            .catch(err => console.error('[VR-ARM] Failed to fetch arm position:', err));
    },

    sendGripperCommand: function (closed) {
        console.log('[VR-GRIPPER] Sending gripper command:', closed ? 'CLOSE' : 'OPEN');
        fetch('/gripper', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ closed: closed })
        })
            .then(res => res.json())
            .then(data => console.log('[VR-GRIPPER] Response:', data))
            .catch(err => console.error('[VR-GRIPPER] Gripper command failed:', err));
    },

    sendGripRelease: function (hand) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify({
                hand: hand,
                gripReleased: true
            }));
        }
    },

    updateStatus: function (text, color) {
        if (this.statusText) {
            this.statusText.setAttribute('value', text);
            this.statusText.setAttribute('color', color);
        }
    },

    updateDesktopStatus: function () {
        const vrIndicator = document.querySelector('#vrStatus');
        const robotIndicator = document.querySelector('#robotStatus');

        if (vrIndicator) {
            vrIndicator.classList.toggle('connected', vrConnected);
        }
        if (robotIndicator) {
            fetch('/status')
                .then(res => res.json())
                .then(data => {
                    robotConnected = data.controller_connected;
                    robotIndicator.classList.toggle('connected', robotConnected);
                })
                .catch(() => { });
        }
    },

    calculateZAxisRotation: function (currentQuat, initialQuat) {
        // Calculate relative quaternion
        const relativeQuat = new THREE.Quaternion();
        relativeQuat.multiplyQuaternions(currentQuat, initialQuat.clone().invert());

        // Get rotation around local Z-axis (forward)
        const forwardDir = new THREE.Vector3(0, 0, 1);
        forwardDir.applyQuaternion(currentQuat);

        const angle = 2 * Math.acos(Math.abs(relativeQuat.w));
        if (angle < 0.0001) return 0;

        const sinHalf = Math.sqrt(1 - relativeQuat.w * relativeQuat.w);
        const rotAxis = new THREE.Vector3(
            relativeQuat.x / sinHalf,
            relativeQuat.y / sinHalf,
            relativeQuat.z / sinHalf
        );

        const projected = rotAxis.dot(forwardDir);
        let degrees = THREE.MathUtils.radToDeg(angle * projected);

        // Normalize to -180 to 180
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;

        return degrees;
    },

    tick: function () {
        if (!this.leftHand || !this.rightHand) return;

        // Collect controller data
        const leftController = this.collectControllerData(this.leftHand, 'left', this.leftGripDown, this.leftTriggerDown);
        const rightController = this.collectControllerData(this.rightHand, 'right', this.rightGripDown, this.rightTriggerDown);

        // Collect headset data
        const headset = this.collectHeadsetData();

        // Update controller info text
        this.updateControllerInfoText(leftController, this.leftHandInfoText, 'L');
        this.updateControllerInfoText(rightController, this.rightHandInfoText, 'R');

        // Process arm control (right controller with grip held)
        if (this.rightGripDown && this.rightGripInitialPosition && rightController.position) {
            this.processArmControl(rightController);
        }

        // Process movement control (left thumbstick)
        if (leftController.thumbstick) {
            this.processMovementControl(leftController.thumbstick);
        }

        // Send data via WebSocket
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            const hasValidData = leftController.position || rightController.position;

            if (hasValidData) {
                this.websocket.send(JSON.stringify({
                    timestamp: Date.now(),
                    leftController: leftController,
                    rightController: rightController,
                    headset: headset
                }));
            }
        }
    },

    collectControllerData: function (hand, handName, gripActive, triggerActive) {
        const data = {
            hand: handName,
            position: null,
            rotation: null,
            quaternion: null,
            gripActive: gripActive,
            trigger: triggerActive ? 1 : 0,
            thumbstick: null,
            buttons: null
        };

        if (!hand || !hand.object3D) return data;

        const pos = hand.object3D.position;
        const rot = hand.object3D.rotation;
        const quat = hand.object3D.quaternion;

        data.position = { x: pos.x, y: pos.y, z: pos.z };
        data.rotation = {
            x: THREE.MathUtils.radToDeg(rot.x),
            y: THREE.MathUtils.radToDeg(rot.y),
            z: THREE.MathUtils.radToDeg(rot.z)
        };
        data.quaternion = { x: quat.x, y: quat.y, z: quat.z, w: quat.w };

        // Get thumbstick and buttons from gamepad
        if (hand.components && hand.components['tracked-controls']) {
            const gamepad = hand.components['tracked-controls'].controller?.gamepad;
            if (gamepad) {
                data.thumbstick = {
                    x: gamepad.axes[2] || 0,
                    y: gamepad.axes[3] || 0
                };
                data.buttons = {
                    a: !!gamepad.buttons[3]?.pressed,
                    b: !!gamepad.buttons[4]?.pressed
                };
            }
        }

        return data;
    },

    collectHeadsetData: function () {
        const data = { position: null, rotation: null, quaternion: null };

        if (!this.headset || !this.headset.object3D) return data;

        const pos = this.headset.object3D.position;
        const rot = this.headset.object3D.rotation;
        const quat = this.headset.object3D.quaternion;

        data.position = { x: pos.x, y: pos.y, z: pos.z };
        data.rotation = {
            x: THREE.MathUtils.radToDeg(rot.x),
            y: THREE.MathUtils.radToDeg(rot.y),
            z: THREE.MathUtils.radToDeg(rot.z)
        };
        data.quaternion = { x: quat.x, y: quat.y, z: quat.z, w: quat.w };

        return data;
    },

    updateControllerInfoText: function (controller, textEl, prefix) {
        if (!textEl || !controller.position) return;

        const pos = controller.position;
        const text = `${prefix}: ${pos.x.toFixed(2)} ${pos.y.toFixed(2)} ${pos.z.toFixed(2)}`;
        textEl.setAttribute('value', text);
    },

    processArmControl: function (controller) {
        // Throttle updates to prevent servo bus saturation
        const now = Date.now();
        if (now - this.lastArmUpdate < this.armUpdateInterval) return;
        this.lastArmUpdate = now;

        if (!this.rightGripInitialPosition) return;

        const currentPos = this.rightHand.object3D.position;
        const currentQuat = this.rightHand.object3D.quaternion;

        // Calculate relative position delta (in meters)
        // Quest coordinate system: X = left(-)/right(+), Y = down(-)/up(+), Z = back(-)/forward(+)
        const deltaX = currentPos.x - this.rightGripInitialPosition.x;
        const deltaY = currentPos.y - this.rightGripInitialPosition.y;
        const deltaZ = currentPos.z - this.rightGripInitialPosition.z;

        // Calculate wrist roll from controller rotation
        let wristRollDelta = 0;
        if (this.rightGripInitialQuaternion) {
            wristRollDelta = this.calculateZAxisRotation(currentQuat, this.rightGripInitialQuaternion);
        }

        // Arm mapping:
        // - Hand LEFT/RIGHT (X) → shoulder_pan (arm swings left/right)
        // - Hand UP/DOWN (Y) → shoulder_lift (arm raises/lowers)
        // - Hand FORWARD/BACK (Z) → elbow_flex (arm extends/retracts) + some shoulder_lift
        const armTarget = {
            shoulder_pan: this.clamp(this.armBasePosition.shoulder_pan - deltaX * this.armScale.pan, -90, 90),
            shoulder_lift: this.clamp(this.armBasePosition.shoulder_lift + deltaY * this.armScale.lift - deltaZ * 30, -90, 90),
            elbow_flex: this.clamp(this.armBasePosition.elbow_flex - deltaZ * this.armScale.reach, -90, 90),
            wrist_flex: this.armBasePosition.wrist_flex,
            wrist_roll: this.clamp(this.armBasePosition.wrist_roll + wristRollDelta * (this.armScale.wristRoll / 90), -150, 150)
        };

        // Debug log every 500ms
        if (!this._lastArmLog || now - this._lastArmLog > 500) {
            console.log(`[VR-ARM] Delta: X=${deltaX.toFixed(3)} Y=${deltaY.toFixed(3)} Z=${deltaZ.toFixed(3)}`);
            console.log(`[VR-ARM] Target: pan=${armTarget.shoulder_pan.toFixed(1)} lift=${armTarget.shoulder_lift.toFixed(1)} elbow=${armTarget.elbow_flex.toFixed(1)}`);
            this._lastArmLog = now;
        }

        // Send arm command
        fetch('/arm', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ positions: armTarget })
        })
            .then(res => res.json())
            .then(data => {
                if (data.error) {
                    console.error('[VR-ARM] Arm error:', data.error);
                }
            })
            .catch(err => console.error('[VR-ARM] Arm command failed:', err));
    },

    processMovementControl: function (thumbstick) {
        // Throttle movement updates
        const now = Date.now();
        if (now - this.lastMoveUpdate < this.moveUpdateInterval) return;
        this.lastMoveUpdate = now;

        const x = thumbstick.x;
        const y = thumbstick.y;

        // Apply deadzone
        const magnitude = Math.sqrt(x * x + y * y);
        if (magnitude < this.thumbstickDeadzone) {
            // Stop movement if within deadzone (only send once when stopping)
            if (this.isMoving) {
                this.isMoving = false;
                fetch('/move', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        forward: false,
                        backward: false,
                        left: false,
                        right: false
                    })
                }).catch(() => { });
            }
            return;
        }
        this.isMoving = true;

        // Map thumbstick to movement
        // Y axis: forward (negative Y) / backward (positive Y) - Quest thumbstick is inverted
        // X axis: turn left (negative X) / turn right (positive X)
        const movement = {
            forward: y < -this.thumbstickDeadzone,
            backward: y > this.thumbstickDeadzone,
            left: x < -this.thumbstickDeadzone,
            right: x > this.thumbstickDeadzone
        };

        fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(movement)
        }).catch(err => console.error('Movement command failed:', err));
    },

    clamp: function (value, min, max) {
        return Math.max(min, Math.min(max, value));
    }
});

// VR session event handling
document.addEventListener('DOMContentLoaded', () => {
    const scene = document.querySelector('a-scene');
    const enterVrBtn = document.getElementById('enterVrBtn');
    const xrStatus = document.getElementById('xrStatus');

    // Custom WebXR Support Check
    const checkWebXR = async () => {
        if (enterVrBtn) enterVrBtn.style.display = 'block'; // Always show button

        if (!navigator.xr) {
            if (xrStatus) xrStatus.textContent = "WebXR not available (Check HTTPS/Flags)";
            if (xrStatus) xrStatus.style.color = "#FF9800";
            return;
        }

        try {
            const supported = await navigator.xr.isSessionSupported('immersive-vr');
            if (supported) {
                if (xrStatus) xrStatus.textContent = "WebXR Ready";
                if (xrStatus) xrStatus.style.color = "#4CAF50";
            } else {
                if (xrStatus) xrStatus.textContent = "VR session not supported on this device";
            }
        } catch (err) {
            console.error("WebXR check failed:", err);
            if (xrStatus) xrStatus.textContent = `WebXR Error: ${err.message}`;
            if (xrStatus) xrStatus.style.color = "#f44336";
        }
    };

    checkWebXR();

    if (enterVrBtn) {
        enterVrBtn.onclick = () => {
            if (scene) {
                // Try entering VR even if check failed (might work with polyfill/shim)
                scene.enterVR();
            }
        };
    }

    if (scene) {
        scene.addEventListener('enter-vr', () => {
            console.log('Entered VR mode');
            vrConnected = true;
            document.querySelector('#vrStatus')?.classList.add('connected');
        });

        scene.addEventListener('exit-vr', () => {
            console.log('Exited VR mode');
            vrConnected = false;
            document.querySelector('#vrStatus')?.classList.remove('connected');

            // Stop all movement when exiting VR
            fetch('/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    forward: false,
                    backward: false,
                    left: false,
                    right: false
                })
            }).catch(() => { });
        });

        scene.addEventListener('controllerconnected', (evt) => {
            console.log('Controller connected:', evt.detail.name, evt.detail.component?.data?.hand);
        });

        scene.addEventListener('controllerdisconnected', (evt) => {
            console.log('Controller disconnected:', evt.detail.name);
        });
    }

    // Initial status check
    fetch('/status')
        .then(res => res.json())
        .then(data => {
            document.querySelector('#robotStatus')?.classList.toggle('connected', data.controller_connected);
        })
        .catch(() => { });
});
