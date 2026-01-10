/**
 * ARCS System Diagnostics Board
 * Handles real-time data visualization and status updates.
 */

class Dashboard {
    constructor() {
        // UI References
        this.modeBadge = document.getElementById('mode-badge');
        this.clockEl = document.getElementById('system-clock');

        // Status Indicators
        this.statusController = document.getElementById('status-controller');
        this.statusArm = document.getElementById('status-arm');
        this.statusCamera = document.getElementById('status-camera');
        this.statusLidar = document.getElementById('status-lidar');

        // Metrics
        this.lidarBar = document.getElementById('lidar-bar');
        this.lidarValue = document.getElementById('lidar-value');

        // Core Animation
        this.coreHeart = document.getElementById('core-heart');
        this.coreIcon = document.getElementById('core-icon');
        this.coreStatusText = document.getElementById('core-status-text');
        this.coreRingInner = document.querySelector('.core-ring.inner');

        // Logs & Task
        this.logTerminal = document.getElementById('log-terminal');
        this.taskContent = document.getElementById('task-content');

        // Warnings
        this.warnLeft = document.getElementById('warn-left');
        this.warnCenter = document.getElementById('warn-center');
        this.warnRight = document.getElementById('warn-right');

        // State
        this.currentMode = 'idle';
        this.lastLogs = new Set(); // To avoid dupes/spam if needed
        this.logHistory = [];

        this.init();
    }

    init() {
        this.startClock();
        this.pollState();
    }

    startClock() {
        setInterval(() => {
            const now = new Date();
            this.clockEl.textContent = now.toLocaleTimeString('en-US', { hour12: false });
        }, 1000);
    }

    async pollState() {
        try {
            const response = await fetch('/display/state');
            if (response.ok) {
                const data = await response.json();
                this.updateUI(data);
            }
        } catch (e) {
            console.error("Poll failed", e);
            this.setOffline();
        }
        setTimeout(() => this.pollState(), 500);
    }

    updateUI(data) {
        // 1. Connection Status
        this.setIndicator(this.statusController, data.controller_connected);
        this.setIndicator(this.statusArm, data.arm_connected);
        this.setIndicator(this.statusCamera, data.camera_connected);
        this.setIndicator(this.statusLidar, data.lidar_connected);

        // 2. Metrics (Lidar)
        if (data.lidar_distance !== null && data.lidar_distance !== undefined) {
            const dist = data.lidar_distance;
            const maxDist = 200; // cm for full bar roughly
            const percent = Math.min(100, Math.max(0, (dist / maxDist) * 100));

            this.lidarBar.style.width = `${percent}%`;
            this.lidarValue.textContent = `${dist.toFixed(1)} cm`;

            // Color coding based on distance
            if (dist < 30) this.lidarBar.style.backgroundColor = 'var(--accent-red)';
            else if (dist < 80) this.lidarBar.style.backgroundColor = 'var(--accent-blue)';
            else this.lidarBar.style.backgroundColor = 'var(--accent-green)';
        } else {
            this.lidarBar.style.width = '0%';
            this.lidarValue.textContent = '--- cm';
        }

        // 3. Control Mode & Core Config
        this.updateMode(data);

        // 4. Task Display
        if (data.current_task) {
            this.taskContent.textContent = data.current_task;
        } else if (data.ai_status && data.ai_status !== 'Idle') {
            this.taskContent.textContent = data.ai_status;
        } else {
            this.taskContent.textContent = "System Idle. Awaiting instructions.";
        }

        // 5. Blockage Warnings
        if (data.blockage) {
            this.toggleWarning(this.warnLeft, data.blockage.left);
            this.toggleWarning(this.warnCenter, data.blockage.forward);
            this.toggleWarning(this.warnRight, data.blockage.right);
        }

        // 6. Logs - Simulating logs from AI status changes for now since we don't have a direct log stream in the state object yet, 
        // though we could fetch /api/logs if we wanted full detail. For visual effect, we'll just log interesting state changes.
        if (data.ai_status && data.ai_status !== this.lastAiStatus) {
            this.addLog(`[AI] ${data.ai_status}`);
            this.lastAiStatus = data.ai_status;
        }
    }

    updateMode(data) {
        let mode = 'IDLE';
        let color = '--accent-blue';
        let icon = '⚡';
        let statusText = 'SYSTEM READY';

        if (data.ai_enabled) {
            mode = 'AI CONTROL';
            color = '--accent-purple';
            icon = '🧠';
            statusText = 'AI ACTIVE';

            if (data.precision_mode) {
                statusText = 'PRECISION MODE';
                icon = '🎯';
            }
        } else if (data.control_mode === 'remote') {
            mode = 'REMOTE';
            color = '--accent-green';
            icon = '🎮';
            statusText = 'MANUAL OVERRIDE';
        }

        // Update Text/Badge
        this.modeBadge.textContent = mode;
        this.modeBadge.className = 'mode-badge'; // reset
        if (mode === 'AI CONTROL') this.modeBadge.classList.add('ai');
        if (mode === 'REMOTE') this.modeBadge.classList.add('remote');

        // Update Core
        this.coreHeart.style.backgroundColor = `var(${color})`;
        this.coreHeart.style.boxShadow = `0 0 30px var(${color})`;
        this.coreRingInner.style.borderColor = `var(${color}) transparent var(${color}) transparent`;
        this.coreIcon.textContent = icon;
        this.coreStatusText.textContent = statusText;
        this.coreStatusText.style.textShadow = `0 0 10px var(${color})`;
    }

    setIndicator(el, active) {
        if (active) {
            el.classList.add('active');
            el.classList.remove('error');
        } else {
            el.classList.remove('active');
            el.classList.add('error');
        }
    }

    toggleWarning(el, visible) {
        if (visible) el.classList.add('visible');
        else el.classList.remove('visible');
    }

    setOffline() {
        this.modeBadge.textContent = 'OFFLINE';
        this.coreStatusText.textContent = 'CONNECTION LOST';
        this.coreHeart.style.backgroundColor = 'var(--text-secondary)';
        this.coreHeart.style.animation = 'none';
        this.lidarValue.textContent = "OFFLINE";
    }

    addLog(msg) {
        const entry = document.createElement('div');
        entry.className = 'log-entry';

        const time = new Date().toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
        entry.innerHTML = `<span class="timestamp">[${time}]</span> ${msg}`;

        this.logTerminal.appendChild(entry);

        // Keep only last 8 logs
        while (this.logTerminal.children.length > 8) {
            this.logTerminal.removeChild(this.logTerminal.firstChild);
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.dashboard = new Dashboard();
});
