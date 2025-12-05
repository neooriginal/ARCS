/**
 * RoboCrew Control - Client-side JavaScript
 */

// DOM Elements
const videoContainer = document.getElementById('video-container');
const yawDisplay = document.getElementById('yaw-value');
const pitchDisplay = document.getElementById('pitch-value');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const connectionDot = document.getElementById('connection-dot');
const debugPanel = document.getElementById('debug-panel');
const compassArrow = document.getElementById('compass-arrow');

// State - will be set from server on init
let mouseLocked = false;
let currentYaw = null;
let currentPitch = null;
let keysPressed = { w: false, a: false, s: false, d: false };
let lastHeadUpdate = 0;
let headUpdatePending = false;
let controllerConnected = false;
let initialized = false;
let baselineYaw = 0;  // Calibration: yaw value when body is straight

// Settings - optimized for low latency
const MOUSE_SENS = 0.15;
const YAW_MIN = -180, YAW_MAX = 180;
const PITCH_MIN = -180, PITCH_MAX = 180;
const HEAD_UPDATE_INTERVAL = 33; // ~30 updates/sec for smooth control

function showDebug(msg) {
    debugPanel.textContent = msg;
    debugPanel.classList.add('show');
    console.log('[DEBUG]', msg);
}

function hideDebug() {
    debugPanel.classList.remove('show');
}

function updateDisplay() {
    if (currentYaw !== null) {
        yawDisplay.textContent = currentYaw.toFixed(1) + '°';
        updateCompass();
    }
    if (currentPitch !== null) {
        pitchDisplay.textContent = currentPitch.toFixed(1) + '°';
    }
}

function updateCompass() {
    // Arrow shows where body is facing relative to camera
    const relativeYaw = currentYaw - baselineYaw;
    compassArrow.style.transform = `translate(-50%, -100%) rotate(${-relativeYaw}deg)`;
}

function updateStatus(text, state) {
    statusText.textContent = text;
    statusDot.className = 'status-dot' + (state ? ' ' + state : '');
}

// Initialize - read current head position from robot
async function init() {
    updateStatus('Connecting...', '');
    
    try {
        const res = await fetch('/head_position');
        const data = await res.json();
        
        if (data.error) {
            showDebug('Head position error: ' + data.error);
            connectionDot.classList.add('error');
            updateStatus('Error', 'error');
            return;
        }
        
        // Use actual robot position as starting point
        currentYaw = data.yaw;
        currentPitch = data.pitch;
        baselineYaw = data.yaw;  // This is our "straight ahead" calibration
        controllerConnected = true;
        initialized = true;
        
        updateDisplay();
        connectionDot.classList.remove('error');
        updateStatus('Ready', 'active');
        
        showDebug('Robot position: Yaw=' + currentYaw + ', Pitch=' + currentPitch);
        console.log('Initialized with robot position:', currentYaw, currentPitch);
        
        // Auto-hide debug after 3 seconds
        setTimeout(hideDebug, 3000);
        
    } catch(e) {
        showDebug('Connection error: ' + e.message);
        connectionDot.classList.add('error');
        updateStatus('Offline', 'error');
    }
}

init();

// Pointer Lock for FPS-style mouse control
videoContainer.addEventListener('click', () => {
    if (!mouseLocked && initialized) {
        videoContainer.requestPointerLock();
    }
});

document.addEventListener('pointerlockchange', () => {
    mouseLocked = document.pointerLockElement === videoContainer;
    videoContainer.classList.toggle('locked', mouseLocked);
    if (mouseLocked) {
        updateStatus('Controlling', 'active');
    } else {
        updateStatus('Ready', 'active');
    }
});

// Throttled head position update with request cancellation
let headAbortController = null;

function scheduleHeadUpdate() {
    if (!initialized) return;
    
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
    
    // Cancel any pending request
    if (headAbortController) {
        headAbortController.abort();
    }
    headAbortController = new AbortController();
    
    lastHeadUpdate = Date.now();
    try {
        await fetch('/head', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ yaw: currentYaw, pitch: currentPitch }),
            signal: headAbortController.signal
        });
        // Don't wait for response parsing - fire and forget for speed
    } catch(e) {
        if (e.name !== 'AbortError') {
            console.log('Head update error:', e.message);
        }
    }
}

// Mouse movement handler
document.addEventListener('mousemove', (e) => {
    if (!mouseLocked || !initialized) return;
    
    const deltaYaw = e.movementX * MOUSE_SENS;
    const deltaPitch = e.movementY * MOUSE_SENS;
    
    currentYaw = Math.max(YAW_MIN, Math.min(YAW_MAX, currentYaw + deltaYaw));
    currentPitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, currentPitch + deltaPitch));
    
    updateDisplay();
    scheduleHeadUpdate();
});

// Keyboard controls (WASD only)
function updateKeyDisplay() {
    ['w', 'a', 's', 'd'].forEach(key => {
        const el = document.getElementById('key-' + key);
        if (el) el.classList.toggle('active', keysPressed[key]);
    });
}

async function sendMovement() {
    const isMoving = Object.values(keysPressed).some(v => v);
    
    if (isMoving) {
        updateStatus('Moving', 'moving');
    } else if (mouseLocked) {
        updateStatus('Controlling', 'active');
    } else {
        updateStatus('Ready', 'active');
    }
    
    try {
        const res = await fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                forward: keysPressed.w,
                backward: keysPressed.s,
                left: keysPressed.a,
                right: keysPressed.d
            })
        });
        const data = await res.json();
        if (data.status === 'error') {
            showDebug('Move error: ' + data.error);
            updateStatus('Error', 'error');
        }
    } catch(e) {
        showDebug('Move request failed: ' + e.message);
    }
}

document.addEventListener('keydown', (e) => {
    if (!initialized) return;
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key) && !keysPressed[key]) {
        keysPressed[key] = true;
        updateKeyDisplay();
        sendMovement();
    }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key)) {
        keysPressed[key] = false;
        updateKeyDisplay();
        sendMovement();
    }
});

// Handle window blur - stop movement
window.addEventListener('blur', () => {
    keysPressed = { w: false, a: false, s: false, d: false };
    updateKeyDisplay();
    sendMovement();
});
