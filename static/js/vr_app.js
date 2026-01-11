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
        this.sendInterval = 50;
        this.hfLoggedIn = false;

        this.connectSocket();
        this.setupEvents();
        this.setupRecording();
        this.setupArmModel();
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

            this.socket.on('arm_state', (data) => {
                if (data && data.angles) {
                    this.updateArmModel(data.angles);
                }
            });
        } catch (e) {
            console.error('Socket error:', e);
        }
    },

    setupArmModel: function () {
        this.armModelEl = document.querySelector('#armModel');
        this.fetchArmPosition();
        setInterval(() => this.fetchArmPosition(), 500);
    },

    fetchArmPosition: async function () {
        try {
            const res = await fetch('/arm_position');
            const data = await res.json();
            if (data.positions) {
                const p = data.positions;
                const angles = [
                    p.shoulder_pan || 0,
                    p.shoulder_lift || 0,
                    p.elbow_flex || 0,
                    p.wrist_flex || 0,
                    p.wrist_roll || 0,
                    p.gripper || 0
                ];
                this.updateArmModel(angles);
            }
        } catch (e) { }
    },

    updateArmModel: function (angles) {
        if (this.armModelEl && this.armModelEl.components['vr-arm-model']) {
            this.armModelEl.components['vr-arm-model'].setJointAngles(angles);
        }
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
