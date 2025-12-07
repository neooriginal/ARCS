/**
 * RoboCrew Display Visualization
 * Animated eyes and status display
 */

class RoboDisplay {
    constructor() {
        this.leftIris = document.getElementById('left-iris');
        this.rightIris = document.getElementById('right-iris');
        this.leftEye = document.getElementById('left-eye');
        this.rightEye = document.getElementById('right-eye');
        this.statusDot = document.getElementById('status-dot');
        this.statusLabel = document.getElementById('status-label');
        this.taskDisplay = document.getElementById('task-display');

        // Control mode elements
        this.controlModeBadge = document.getElementById('control-mode-badge');
        this.modeEmoji = document.getElementById('mode-emoji');
        this.modeText = document.getElementById('mode-text');

        // System status elements
        this.systemController = document.getElementById('system-controller');
        this.systemCamera = document.getElementById('system-camera');
        this.systemArm = document.getElementById('system-arm');
        this.systemAI = document.getElementById('system-ai');

        this.currentExpression = 'idle';
        this.currentControlMode = 'idle';
        this.isBlinking = false;
        this.lookTarget = { x: 0, y: 0 };
        this.currentLook = { x: 0, y: 0 };

        this.init();
    }

    init() {
        // Start animations
        this.startBlinkLoop();
        this.startLookAroundLoop();
        this.startStatusPolling();
        this.animate();
    }

    // Eye movement
    setLookTarget(x, y) {
        this.lookTarget.x = Math.max(-1, Math.min(1, x));
        this.lookTarget.y = Math.max(-1, Math.min(1, y));
    }

    updateEyePosition() {
        this.currentLook.x += (this.lookTarget.x - this.currentLook.x) * 0.1;
        this.currentLook.y += (this.lookTarget.y - this.currentLook.y) * 0.1;

        const maxOffset = 25;
        const offsetX = this.currentLook.x * maxOffset;
        const offsetY = this.currentLook.y * maxOffset;

        this.leftIris.style.transform = `translate(${offsetX}px, ${offsetY}px)`;
        this.rightIris.style.transform = `translate(${offsetX}px, ${offsetY}px)`;
    }

    // Blinking
    blink() {
        if (this.isBlinking) return;
        this.isBlinking = true;

        this.leftEye.classList.add('blinking');
        this.rightEye.classList.add('blinking');

        setTimeout(() => {
            this.leftEye.classList.remove('blinking');
            this.rightEye.classList.remove('blinking');
            this.isBlinking = false;
        }, 150);
    }

    startBlinkLoop() {
        const doBlink = () => {
            if (this.currentExpression !== 'happy') {
                this.blink();
            }
            const nextBlink = 2000 + Math.random() * 4000;
            setTimeout(doBlink, nextBlink);
        };
        setTimeout(doBlink, 1000);
    }

    // Look around randomly
    startLookAroundLoop() {
        const lookAround = () => {
            if (this.currentExpression === 'idle' || this.currentExpression === 'active') {
                const x = (Math.random() - 0.5) * 1.5;
                const y = (Math.random() - 0.5) * 0.8;
                this.setLookTarget(x, y);
            }
            const nextLook = 1000 + Math.random() * 3000;
            setTimeout(lookAround, nextLook);
        };
        setTimeout(lookAround, 2000);
    }

    // Expressions
    setExpression(expression) {
        if (this.currentExpression === expression) return;

        const expressions = ['happy', 'thinking', 'error'];
        expressions.forEach(exp => {
            this.leftEye.classList.remove(exp);
            this.rightEye.classList.remove(exp);
        });

        if (expressions.includes(expression)) {
            this.leftEye.classList.add(expression);
            this.rightEye.classList.add(expression);
        }

        this.currentExpression = expression;

        if (expression === 'happy') {
            this.setLookTarget(0, 0);
        } else if (expression === 'thinking') {
            this.setLookTarget(0.3, -0.5);
        }
    }

    // Control mode update
    updateControlMode(mode) {
        if (this.currentControlMode === mode) return;
        this.currentControlMode = mode;

        // Remove old mode classes
        this.controlModeBadge.classList.remove('idle', 'remote', 'ai');
        this.controlModeBadge.classList.add(mode);

        // Update emoji and text
        switch (mode) {
            case 'remote':
                this.modeEmoji.textContent = '🎮';
                this.modeText.textContent = 'Remote Control';
                break;
            case 'ai':
                this.modeEmoji.textContent = '🤖';
                this.modeText.textContent = 'AI Driving';
                break;
            case 'idle':
            default:
                this.modeEmoji.textContent = '😴';
                this.modeText.textContent = 'Idle';
                break;
        }
    }

    // Update system status indicators
    updateSystemStatus(data) {
        // Controller
        if (data.controller_connected) {
            this.systemController.classList.add('operational');
            this.systemController.classList.remove('offline');
        } else {
            this.systemController.classList.add('offline');
            this.systemController.classList.remove('operational');
        }

        // Camera
        if (data.camera_connected) {
            this.systemCamera.classList.add('operational');
            this.systemCamera.classList.remove('offline');
        } else {
            this.systemCamera.classList.add('offline');
            this.systemCamera.classList.remove('operational');
        }

        // Arm
        if (data.arm_connected) {
            this.systemArm.classList.add('operational');
            this.systemArm.classList.remove('offline');
        } else {
            this.systemArm.classList.add('offline');
            this.systemArm.classList.remove('operational');
        }

        // AI
        if (data.ai_enabled) {
            this.systemAI.classList.add('operational');
            this.systemAI.classList.remove('offline');
        } else {
            this.systemAI.classList.add('offline');
            this.systemAI.classList.remove('operational');
        }
    }

    // Status polling
    async startStatusPolling() {
        const poll = async () => {
            try {
                const response = await fetch('/display/state');
                if (response.ok) {
                    const data = await response.json();
                    this.updateFromState(data);
                    this.updateSystemStatus(data);
                    this.updateControlMode(data.control_mode || 'idle');
                }
            } catch (error) {
                // Connection lost
                this.systemController.classList.add('offline');
                this.systemCamera.classList.add('offline');
                this.systemArm.classList.add('offline');
                this.systemAI.classList.add('offline');
            }
            setTimeout(poll, 500);
        };
        poll();
    }

    updateFromState(data) {
        this.statusDot.className = 'status-dot';

        if (data.ai_enabled) {
            this.statusDot.classList.add('active');
            this.statusLabel.textContent = 'AI Active';
            this.setExpression('active');
        } else {
            this.statusDot.classList.add('idle');
            this.statusLabel.textContent = 'Idle';
            this.setExpression('idle');
        }

        // Update task display
        if (data.current_task) {
            this.taskDisplay.textContent = data.current_task;
        } else if (data.ai_status && data.ai_status !== 'Idle') {
            this.taskDisplay.textContent = data.ai_status;
        } else {
            this.taskDisplay.textContent = 'Ready to help!';
        }

        // Expression based on status
        if (data.ai_status) {
            const status = data.ai_status.toLowerCase();
            if (status.includes('error') || status.includes('failed')) {
                this.setExpression('error');
                this.statusDot.className = 'status-dot error';
            } else if (status.includes('thinking') || status.includes('planning')) {
                this.setExpression('thinking');
            } else if (status.includes('complete') || status.includes('success') || status.includes('done')) {
                this.setExpression('happy');
                setTimeout(() => {
                    if (this.currentExpression === 'happy') {
                        this.setExpression('idle');
                    }
                }, 3000);
            }
        }
    }

    // Animation loop
    animate() {
        this.updateEyePosition();
        requestAnimationFrame(() => this.animate());
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.roboDisplay = new RoboDisplay();
});
