AFRAME.registerComponent('vr-controller-updater', {
    init: function () {
        this.rightHand = document.querySelector('#rightHand');
        this.leftHand = document.querySelector('#leftHand');
        this.rightInfo = document.querySelector('#rightHandInfo');
        this.leftInfo = document.querySelector('#leftHandInfo');

        this.socket = null;
        this.rightGripDown = false;
        this.rightTriggerDown = false;
        this.leftStick = { x: 0, y: 0 };
        this.lastSend = 0;
        this.sendInterval = 50;

        this.connectSocket();
        this.setupEvents();
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
        } catch (e) {
            console.error('Socket error:', e);
        }
    },

    updateStatus: function (id, on) {
        const el = document.getElementById(id);
        if (el) el.classList.toggle('connected', on);
    },

    setupEvents: function () {
        if (!this.rightHand || !this.leftHand) return;

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
            // Animate visual gripper on controller
            const fingerL = document.querySelector('#fingerL');
            const fingerR = document.querySelector('#fingerR');
            if (fingerL) fingerL.setAttribute('position', '-0.002 0 -0.11');
            if (fingerR) fingerR.setAttribute('position', '0.002 0 -0.11');
        });
        this.rightHand.addEventListener('triggerup', () => {
            this.rightTriggerDown = false;
            // Reset visual gripper
            const fingerL = document.querySelector('#fingerL');
            const fingerR = document.querySelector('#fingerR');
            if (fingerL) fingerL.setAttribute('position', '-0.015 0 -0.11');
            if (fingerR) fingerR.setAttribute('position', '0.015 0 -0.11');

            if (this.socket?.connected) this.socket.emit('vr_data', { triggerReleased: true });
        });

        this.leftHand.addEventListener('thumbstickmoved', e => {
            this.leftStick = { x: e.detail.x, y: e.detail.y };
        });
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
            Math.abs(this.leftStick.x) > 0.1 || Math.abs(this.leftStick.y) > 0.1;

        if (hasInput && right.position) {
            this.socket.emit('vr_data', {
                rightController: right,
                leftController: { thumbstick: this.leftStick }
            });
        }
    }
});
