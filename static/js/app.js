/**
 * RoboCrew Control - Client-side JavaScript
 * Dual-mode control: Drive (WASD + mouse head) and Arm (mouse + keyboard)
 */

// DOM Elements
const videoContainer = document.getElementById('video-container');
const armContainer = document.getElementById('arm-container');
const drivePanel = document.getElementById('drive-panel');
const armPanel = document.getElementById('arm-panel');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const connectionDot = document.getElementById('connection-dot');
const debugPanel = document.getElementById('debug-panel');
const compassArrow = document.getElementById('compass-arrow');

// Arm display elements
const armPanDisplay = document.getElementById('arm-pan');
const armReachDisplay = document.getElementById('arm-reach');
const armWristDisplay = document.getElementById('arm-wrist-val');
const armGripper = document.getElementById('arm-gripper');

// State
let currentMode = 'none'; // 'none', 'drive', 'arm'
let driveLocked = false;
let armLocked = false;

// Drive state
let currentYaw = null;
let currentPitch = null;
let keysPressed = { w: false, a: false, s: false, d: false };
let baselineYaw = 0;

// Arm state
let armConnected = false;
let armPositions = {};
let gripperClosed = false;

// Throttling
let lastHeadUpdate = 0;
let lastArmUpdate = 0;
let headUpdatePending = false;
let armUpdatePending = false;
const HEAD_UPDATE_INTERVAL = 33;
const ARM_UPDATE_INTERVAL = 50;

// Settings
const MOUSE_SENS = 0.15;
const YAW_MIN = -180, YAW_MAX = 180;
const PITCH_MIN = -180, PITCH_MAX = 180;

// ============== Utilities ==============

function showDebug(msg) {
    debugPanel.textContent = msg;
    debugPanel.classList.add('show');
    console.log('[DEBUG]', msg);
}

function hideDebug() {
    debugPanel.classList.remove('show');
}

function updateStatus(text, state) {
    statusText.textContent = text;
    statusDot.className = 'status-dot' + (state ? ' ' + state : '');
}

function updateCompass() {
    if (currentYaw === null) return;
    const relativeYaw = currentYaw - baselineYaw;
    compassArrow.style.transform = `translate(-50%, -100%) rotate(${-relativeYaw}deg)`;
}

function updateArmDisplay() {
    if (armPanDisplay) armPanDisplay.textContent = Math.round(armPositions.shoulder_pan || 0) + '°';
    if (armReachDisplay) armReachDisplay.textContent = Math.round(armPositions.shoulder_lift || 0) + '°';
    if (armWristDisplay) armWristDisplay.textContent = Math.round(armPositions.wrist_roll || 0) + '°';

    // Update gripper visual
    if (armGripper) {
        armGripper.classList.toggle('gripper-closed', gripperClosed);
    }
}

// ============== Initialization ==============

async function init() {
    updateStatus('Connecting...', '');

    try {
        // Get head position
        const headRes = await fetch('/head_position');
        const headData = await headRes.json();

        if (!headData.error) {
            currentYaw = headData.yaw;
            currentPitch = headData.pitch;
            baselineYaw = headData.yaw;
            updateCompass();
        }

        // Get status including arm
        const statusRes = await fetch('/status');
        const status = await statusRes.json();

        armConnected = status.arm_connected;
        if (armConnected) {
            armPositions = status.arm_positions || {};
            updateArmDisplay();
        }

        connectionDot.classList.remove('error');
        updateStatus('Ready', 'active');

        console.log('Initialized:', { headYaw: currentYaw, armConnected });

    } catch (e) {
        showDebug('Connection error: ' + e.message);
        connectionDot.classList.add('error');
        updateStatus('Offline', 'error');
    }
}

init();

// ============== Mode Switching ==============

function setMode(mode) {
    if (currentMode === mode) return;

    // Exit current mode
    if (currentMode === 'drive') {
        document.exitPointerLock();
        driveLocked = false;
        videoContainer.classList.remove('locked');
    } else if (currentMode === 'arm') {
        document.exitPointerLock();
        armLocked = false;
        armContainer.classList.remove('locked');
    }

    // Update panel states
    drivePanel.classList.toggle('active', mode === 'drive');
    armPanel.classList.toggle('active', mode === 'arm');

    currentMode = mode;

    // Notify server
    fetch('/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode })
    }).catch(e => console.log('Mode switch error:', e));

    console.log('Mode:', mode);
}

// Click handlers for panels
videoContainer.addEventListener('click', () => {
    if (currentMode !== 'drive') {
        setMode('drive');
    }
    if (!driveLocked) {
        videoContainer.requestPointerLock();
    }
});

armContainer.addEventListener('click', () => {
    if (!armConnected) {
        showDebug('Arm not connected');
        setTimeout(hideDebug, 2000);
        return;
    }
    if (currentMode !== 'arm') {
        setMode('arm');
    }
    if (!armLocked) {
        armContainer.requestPointerLock();
    }
});

// Pointer lock change
document.addEventListener('pointerlockchange', () => {
    const lockedElement = document.pointerLockElement;

    if (lockedElement === videoContainer) {
        driveLocked = true;
        videoContainer.classList.add('locked');
        updateStatus('Driving', 'active');
    } else if (lockedElement === armContainer) {
        armLocked = true;
        armContainer.classList.add('locked');
        updateStatus('Arm Control', 'active');
    } else {
        driveLocked = false;
        armLocked = false;
        videoContainer.classList.remove('locked');
        armContainer.classList.remove('locked');
        updateStatus('Ready', 'active');
    }
});

// ESC releases and resets mode
document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
        setMode('none');
    }
});

// ============== Drive Mode: Head Control ==============

function scheduleHeadUpdate() {
    const now = Date.now();
    const timeSinceLastUpdate = now - lastHeadUpdate;

    if (timeSinceLastUpdate >= HEAD_UPDATE_INTERVAL) {
        sendHeadUpdate();
    } else if (!headUpdatePending) {
        headUpdatePending = true;
        setTimeout(() => {
            headUpdatePending = false;
            sendHeadUpdate();
        }, HEAD_UPDATE_INTERVAL - timeSinceLastUpdate);
    }
}

async function sendHeadUpdate() {
    if (currentYaw === null || currentPitch === null) return;
    lastHeadUpdate = Date.now();

    try {
        await fetch('/head', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ yaw: currentYaw, pitch: currentPitch })
        });
    } catch (e) {
        console.log('Head update error:', e.message);
    }
}

// ============== Drive Mode: Movement ==============

async function sendMovement() {
    const isMoving = Object.values(keysPressed).some(v => v);

    if (currentMode === 'drive') {
        if (isMoving) {
            updateStatus('Moving', 'moving');
        } else if (driveLocked) {
            updateStatus('Driving', 'active');
        }
    }

    try {
        await fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                forward: keysPressed.w,
                backward: keysPressed.s,
                left: keysPressed.a,
                right: keysPressed.d
            })
        });
    } catch (e) {
        console.log('Move error:', e.message);
    }
}

// ============== Arm Mode: Control ==============

function scheduleArmUpdate(deltaX, deltaY) {
    const now = Date.now();
    const timeSinceLastUpdate = now - lastArmUpdate;

    if (timeSinceLastUpdate >= ARM_UPDATE_INTERVAL) {
        sendArmMouseUpdate(deltaX, deltaY);
    } else if (!armUpdatePending) {
        armUpdatePending = true;
        setTimeout(() => {
            armUpdatePending = false;
            sendArmMouseUpdate(deltaX, deltaY);
        }, ARM_UPDATE_INTERVAL - timeSinceLastUpdate);
    }
}

async function sendArmMouseUpdate(deltaX, deltaY) {
    lastArmUpdate = Date.now();

    try {
        const res = await fetch('/arm/mouse', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ deltaX, deltaY })
        });
        const data = await res.json();
        if (data.positions) {
            armPositions = data.positions;
            updateArmDisplay();
        }
    } catch (e) {
        console.log('Arm mouse error:', e.message);
    }
}

async function sendArmScroll(delta) {
    try {
        const res = await fetch('/arm/scroll', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ delta })
        });
        const data = await res.json();
        if (data.wrist_roll !== undefined) {
            armPositions.wrist_roll = data.wrist_roll;
            updateArmDisplay();
        }
    } catch (e) {
        console.log('Arm scroll error:', e.message);
    }
}

async function sendArmKey(key) {
    try {
        const res = await fetch('/arm/key', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ key })
        });
        const data = await res.json();
        if (data.positions) {
            armPositions = data.positions;
            updateArmDisplay();
        }
    } catch (e) {
        console.log('Arm key error:', e.message);
    }
}

async function setGripper(closed) {
    gripperClosed = closed;
    updateArmDisplay();

    try {
        await fetch('/gripper', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ closed })
        });
    } catch (e) {
        console.log('Gripper error:', e.message);
    }
}

async function armHome() {
    try {
        const res = await fetch('/arm/home', { method: 'POST' });
        const data = await res.json();
        if (data.positions) {
            armPositions = data.positions;
            updateArmDisplay();
        }
    } catch (e) {
        console.log('Arm home error:', e.message);
    }
}

// ============== Event Handlers ==============

// Mouse movement
document.addEventListener('mousemove', (e) => {
    if (currentMode === 'drive' && driveLocked) {
        // Head control
        const deltaYaw = e.movementX * MOUSE_SENS;
        const deltaPitch = e.movementY * MOUSE_SENS;

        currentYaw = Math.max(YAW_MIN, Math.min(YAW_MAX, currentYaw + deltaYaw));
        currentPitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, currentPitch + deltaPitch));

        updateCompass();
        scheduleHeadUpdate();
    } else if (currentMode === 'arm' && armLocked) {
        // Arm control
        scheduleArmUpdate(e.movementX, e.movementY);
    }
});

// Mouse wheel (arm wrist roll)
document.addEventListener('wheel', (e) => {
    if (currentMode === 'arm' && armLocked) {
        e.preventDefault();
        const delta = e.deltaY > 0 ? -1 : 1;
        sendArmScroll(delta);
    }
}, { passive: false });

// Mouse buttons (gripper)
document.addEventListener('mousedown', (e) => {
    if (currentMode === 'arm' && armLocked && e.button === 0) {
        setGripper(true);
    }
});

document.addEventListener('mouseup', (e) => {
    if (currentMode === 'arm' && e.button === 0) {
        setGripper(false);
    }
});

// Keyboard
document.addEventListener('keydown', (e) => {
    const key = e.key.toLowerCase();

    if (currentMode === 'drive') {
        // WASD movement
        if (['w', 'a', 's', 'd'].includes(key) && !keysPressed[key]) {
            keysPressed[key] = true;
            sendMovement();
        }
    } else if (currentMode === 'arm' && armLocked) {
        // Arm keyboard controls
        if (['q', 'e', 'r', 'f', 't', 'g'].includes(key)) {
            sendArmKey(key);
        } else if (key === 'h') {
            armHome();
        }
    }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();

    if (['w', 'a', 's', 'd'].includes(key)) {
        keysPressed[key] = false;
        if (currentMode === 'drive') {
            sendMovement();
        }
    }
});

// Stop movement on window blur
window.addEventListener('blur', () => {
    keysPressed = { w: false, a: false, s: false, d: false };
    sendMovement();
    setGripper(false);
});

// Heartbeat for drive mode: prevent safety timeout while holding keys
setInterval(() => {
    if (currentMode === 'drive' && Object.values(keysPressed).some(v => v)) {
        sendMovement();
    }
}, 200);
