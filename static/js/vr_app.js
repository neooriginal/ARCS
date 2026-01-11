AFRAME.registerComponent('vr-controller-updater', {
    init: function () {
        this.rightHand = document.querySelector('#rightHand');
        this.leftHand = document.querySelector('#leftHand');
        this.rightInfo = document.querySelector('#rightHandInfo');
        this.leftInfo = document.querySelector('#leftHandInfo');

        this.socket = null;
        this.rightGripDown = false;
        this.rightTriggerDown = false;
        this.leftGripDown = false;
        this.leftStick = { x: 0, y: 0 };
        this.rightStick = { x: 0, y: 0 };
        this.lastSend = 0;
        this.sendInterval = 33;  // ~30Hz for responsive tracking
        this.hfLoggedIn = false;

        this.connectSocket();
        this.setupEvents();
        this.setupRecording();
    },

    setupRecording: function () {
        this.recordBtn = document.getElementById('recordBtn');
        this.datasetNameInput = document.getElementById('datasetName');
        this.recordingStatus = document.getElementById('recordingStatus');
        this.recFrameCount = document.getElementById('recFrameCount');
        this.vrRecIndicator = document.getElementById('vrRecIndicator');
        this.isRecording = false;

        const savedName = localStorage.getItem('vr_dataset_name');
        if (savedName) this.datasetNameInput.value = savedName;

        if (this.recordBtn) {
            this.recordBtn.addEventListener('click', () => this.toggleRecording());
        }

        setInterval(() => this.checkRecordingStatus(), 1000);
        setInterval(() => this.checkAuthStatus(), 30000);
        this.checkAuthStatus();
    },

    checkAuthStatus: async function () {
        try {
            const res = await fetch('/api/auth/hf/status');
            const data = await res.json();
            this.hfLoggedIn = data.logged_in;
            this.updateRecordUI(this.isRecording);
        } catch (e) { }
    },

    toggleRecording: async function () {
        if (!this.hfLoggedIn) {
            // Visual feedback?
            return;
        }
        const name = this.datasetNameInput.value.trim();
        if (!name) return;

        localStorage.setItem('vr_dataset_name', name);

        if (!this.isRecording) {
            try {
                const res = await fetch('/api/recording/start', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ dataset_name: name })
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    this.isRecording = true;
                    this.updateRecordUI(true);
                }
            } catch (e) {
                console.error(e);
            }
        } else {
            try {
                const res = await fetch('/api/recording/stop', { method: 'POST' });
                const data = await res.json();
                if (data.status === 'ok') {
                    this.isRecording = false;
                    this.updateRecordUI(false);
                }
            } catch (e) {
                console.error(e);
            }
        }
    },

    checkRecordingStatus: async function () {
        try {
            const res = await fetch('/api/recording/status');
            const data = await res.json();
            this.isRecording = data.is_recording;
            if (this.isRecording) {
                this.updateRecordUI(true);
                if (this.recFrameCount) this.recFrameCount.innerText = data.frame_count;
            } else {
                this.updateRecordUI(false);
            }
        } catch (e) { }
    },

    updateRecordUI: function (recording) {
        if (recording) {
            this.recordingStatus.style.display = 'block';
            this.datasetNameInput.disabled = true;
            if (this.vrRecIndicator) this.vrRecIndicator.setAttribute('visible', true);
        } else {
            this.recordingStatus.style.display = 'none';
            this.datasetNameInput.disabled = false;
            // Show Login Warning if needed
            if (!this.hfLoggedIn) {
                this.recordingStatus.style.display = 'block';
                this.recordingStatus.innerHTML = '<span style="color:orange">⚠️ HF Login Required</span>';
                this.datasetNameInput.disabled = true;
            }
            if (this.vrRecIndicator) this.vrRecIndicator.setAttribute('visible', false);
        }
    },

    connectSocket: function () {
        try {
            this.socket = io(window.location.origin, { transports: ['websocket'] });

            this.socket.on('connect', () => {
                this.updateStatus('wsStatus', true);
                this.socket.emit('vr_connect');
            });
            this.socket.on('disconnect', () => this.updateStatus('wsStatus', false));
            this.socket.on('connect_error', () => this.updateStatus('wsStatus', false));

            // Arm position feedback for 3D visualization
            this.socket.on('arm_position', (pos) => {
                this.updateArmVisualization(pos);
            });
        } catch (e) {
            console.error('Socket error:', e);
        }
    },

    updateArmVisualization: function (pos) {
        const shoulderPan = document.querySelector('#armShoulderPan');
        const shoulderLift = document.querySelector('#armShoulderLift');
        const elbow = document.querySelector('#armElbow');
        const wristFlex = document.querySelector('#armWristFlex');
        const wristRoll = document.querySelector('#armWristRoll');
        const gripperLeft = document.querySelector('#gripperLeft');
        const gripperRight = document.querySelector('#gripperRight');

        if (!shoulderPan) return;

        // Initialize last position storage for noise filtering
        if (!this.lastArmPos) this.lastArmPos = {};

        // Noise deadzone - ignore changes smaller than this
        const deadzone = 0.5;
        const self = this;

        function filter(key, val) {
            if (!(key in self.lastArmPos)) {
                // First update - accept the value as-is
                self.lastArmPos[key] = val;
                return val;
            }
            const last = self.lastArmPos[key];
            if (Math.abs(val - last) < deadzone) return last;
            self.lastArmPos[key] = val;
            return val;
        }

        // Filter positions to reduce jitter
        const sp = filter('sp', pos.shoulder_pan);
        const sl = filter('sl', pos.shoulder_lift);
        const ef = filter('ef', pos.elbow_flex);
        const wf = filter('wf', pos.wrist_flex);
        const wr = filter('wr', pos.wrist_roll);

        // Apply joint rotations
        // Shoulder pan rotates around Y (left/right swing)
        shoulderPan.setAttribute('rotation', `0 ${sp} 0`);
        // Shoulder lift rotates around X (tip forward/back)
        shoulderLift.setAttribute('rotation', `${-sl} 0 0`);
        // Elbow rotates around X - add -90° offset for calibration (neutral = forearm bent forward)
        elbow.setAttribute('rotation', `${-ef - 90} 0 0`);
        // Debug: log elbow value to console
        if (Math.abs(ef) > 1) console.log('Elbow flex:', ef);
        // Wrist flex rotates around X
        wristFlex.setAttribute('rotation', `${-wf} 0 0`);
        // Wrist roll rotates around Y - add 90° offset for calibration
        wristRoll.setAttribute('rotation', `0 ${wr + 90} 0`);

        // Gripper open/close (gripper angle: -65 closed, +80 open)
        const gripperOpen = (pos.gripper + 65) / 145;
        const fingerOffset = 0.012 + gripperOpen * 0.015;
        if (gripperLeft) gripperLeft.setAttribute('position', `${-fingerOffset} 0.04 0`);
        if (gripperRight) gripperRight.setAttribute('position', `${fingerOffset} 0.04 0`);
    },

    updateStatus: function (id, on) {
        const el = document.getElementById(id);
        if (el) el.classList.toggle('connected', on);
    },

    setupEvents: function () {
        if (!this.rightHand || !this.leftHand) return;

        this.rightHand.addEventListener('abuttondown', () => {
            this.toggleRecording();
        });

        this.rightHand.addEventListener('gripdown', () => {
            this.rightGripDown = true;
            this.updateStatus('armStatus', true);
        });
        this.rightHand.addEventListener('gripup', () => {
            this.rightGripDown = false;
            if (this.socket?.connected) this.socket.emit('vr_data', { gripReleased: true });
            this.updateStatus('armStatus', false);
        });

        this.rightHand.addEventListener('triggerdown', () => {
            this.rightTriggerDown = true;
            // Animate moving finger (rotate up to vertical)
            const finger = document.querySelector('#fingerMoving');
            if (finger) finger.setAttribute('animation', 'property: rotation; to: 0 0 0; dur: 200; easing: easeOutQuad');
        });
        this.rightHand.addEventListener('triggerup', () => {
            this.rightTriggerDown = false;
            // Reset moving finger (rotate down to horizontal)
            const finger = document.querySelector('#fingerMoving');
            if (finger) finger.setAttribute('animation', 'property: rotation; to: 0 0 90; dur: 200; easing: easeOutQuad');

            if (this.socket?.connected) this.socket.emit('vr_data', { triggerReleased: true });
        });

        this.rightHand.addEventListener('thumbstickmoved', e => {
            this.rightStick = { x: e.detail.x, y: e.detail.y };
        });

        this.leftHand.addEventListener('thumbstickmoved', e => {
            this.leftStick = { x: e.detail.x, y: e.detail.y };
        });

        this.leftHand.addEventListener('gripdown', () => this.leftGripDown = true);
        this.leftHand.addEventListener('gripup', () => this.leftGripDown = false);
    },

    tick: function () {
        if (!this.rightHand || !this.socket?.connected) return;

        const now = Date.now();
        if (now - this.lastSend < this.sendInterval) return;
        this.lastSend = now;

        const right = {
            position: null, quaternion: null,
            gripActive: this.rightGripDown,
            trigger: this.rightTriggerDown ? 1 : 0
        };

        if (this.rightHand.object3D.visible) {
            const p = this.rightHand.object3D.position;
            const q = this.rightHand.object3D.quaternion;
            right.position = { x: p.x, y: p.y, z: p.z };
            right.quaternion = { x: q.x, y: q.y, z: q.z, w: q.w };

            if (this.rightInfo) {
                this.rightInfo.setAttribute('value',
                    this.rightGripDown ? 'GRIP' : (this.rightTriggerDown ? 'CLOSE' : 'Ready')
                );
            }
        }

        if (this.leftInfo) {
            const { x, y } = this.leftStick;
            this.leftInfo.setAttribute('value',
                (Math.abs(x) > 0.1 || Math.abs(y) > 0.1) ? 'Moving' : 'Move'
            );
        }

        const hasInput = right.gripActive || right.trigger > 0 ||
            Math.abs(this.leftStick.x) > 0.1 || Math.abs(this.leftStick.y) > 0.1 ||
            Math.abs(this.rightStick.x) > 0.1 || Math.abs(this.rightStick.y) > 0.1;

        if (hasInput && right.position) {
            this.socket.emit('vr_data', {
                rightController: {
                    ...right,
                    thumbstick: this.rightStick
                },
                leftController: {
                    thumbstick: this.leftStick,
                    gripActive: this.leftGripDown
                }
            });
        }
    }
});
