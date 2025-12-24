
/**
 * RoboCrew WebXR Control Logic
 * Handles VR session, controller tracking, and maps inputs to robot API.
 */

let xrSession = null;
let xrReferenceSpace = null;
let animationFrameId = null;

// Throttle configuration
const UPDATE_INTERVAL_MS = 100; // Send updates every 100ms
let lastMoveUpdate = 0;
let lastArmUpdate = 0;
let lastHeadUpdate = 0;

// State tracking
const state = {
    move: { x: 0, y: 0, rot: 0 }, // Robot base: x (strafe), y (fwd/back), rot (turn)
    head: { yaw: 0, pitch: 0 },
    arm: {
        shoulder_pan: 0,
        shoulder_lift: 0,
        elbow_flex: 0,
        wrist_flex: 0,
        wrist_roll: 0
    },
    gripper: false
};

const vrButton = document.getElementById('vr-button');
const statusDiv = document.getElementById('status');

// Helper to update status
function setStatus(msg) {
    statusDiv.textContent = msg;
    console.log("[WebXR Status] " + msg);
}

// 1. Check for WebXR support
if (navigator.xr) {
    navigator.xr.isSessionSupported('immersive-vr')
        .then((supported) => {
            if (supported) {
                vrButton.disabled = false;
                vrButton.textContent = "Enter VR";
                vrButton.addEventListener('click', onButtonClicked);
                setStatus("Ready to enter VR");
            } else {
                setStatus("WebXR not supported on this device/browser.");
            }
        });
} else {
    setStatus("WebXR API not available (Requires HTTPS or localhost).");
}

function onButtonClicked() {
    if (!xrSession) {
        navigator.xr.requestSession('immersive-vr', {
            optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking']
        }).then(onSessionStarted)
            .catch(err => {
                setStatus("Failed to start session: " + err.message);
            });
    } else {
        xrSession.end();
    }
}

function onSessionStarted(session) {
    xrSession = session;
    vrButton.textContent = "Exit VR";
    setStatus("VR Session Active");

    session.addEventListener('end', onSessionEnded);

    // Get reference space
    session.requestReferenceSpace('local').then((refSpace) => {
        xrReferenceSpace = refSpace;
        animationFrameId = session.requestAnimationFrame(onXRFrame);
    });
}

function onSessionEnded() {
    xrSession = null;
    vrButton.textContent = "Enter VR";
    setStatus("VR Session Ended");
}

function onXRFrame(time, frame) {
    const session = frame.session;
    animationFrameId = session.requestAnimationFrame(onXRFrame);

    // Get Input Sources (Controllers)
    for (const source of session.inputSources) {
        if (!source.gamepad) continue;

        if (source.handedness === 'left') {
            handleLeftController(source);
        } else if (source.handedness === 'right') {
            handleRightController(source, frame);
        }
    }
}

// --- Controller Handlers ---

function handleLeftController(source) {
    const gp = source.gamepad;
    if (!gp.axes) return;

    // Standard mapping: axes[2] = X (Left/Right), axes[3] = Y (Up/Down)
    const x = gp.axes[2] || 0;
    const y = gp.axes[3] || 0;

    // Deadzone
    const DEADZONE = 0.1;
    const finalX = Math.abs(x) > DEADZONE ? x : 0;
    const finalY = Math.abs(y) > DEADZONE ? y : 0;

    const now = Date.now();
    if (now - lastMoveUpdate > UPDATE_INTERVAL_MS) {
        let direction = 'stop';
        if (finalY < -0.3) direction = 'forward';
        else if (finalY > 0.3) direction = 'backward';
        else if (finalX < -0.3) direction = 'left';
        else if (finalX > 0.3) direction = 'right';

        fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ direction: direction })
        }).catch(e => console.error("Move fetch error:", e));

        lastMoveUpdate = now;
    }
}

function handleRightController(source, frame) {
    // 1. Inputs (Buttons/Stick)
    const gp = source.gamepad;

    // Gripper (Trigger Button - usually button 0)
    const triggerPressed = gp.buttons[0]?.pressed || false;

    // Head Control (Stick)
    const stickX = gp.axes[2] || 0;
    const stickY = gp.axes[3] || 0;

    // 2. Pose (Arm Puppeteering)
    const pose = frame.getPose(source.gripSpace, xrReferenceSpace);

    const now = Date.now();

    // Check Gripper
    if (triggerPressed !== state.gripper && (now - lastArmUpdate > 500)) {
        state.gripper = triggerPressed;
        fetch('/gripper', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ closed: triggerPressed })
        }).catch(e => console.error("Gripper fetch error:", e));
        lastArmUpdate = now;
    }

    // Update Head
    if ((Math.abs(stickX) > 0.1 || Math.abs(stickY) > 0.1) && (now - lastHeadUpdate > 200)) {
        const targetYaw = stickX * -90;
        const targetPitch = stickY * -60;

        fetch('/head', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ yaw: targetYaw, pitch: targetPitch })
        }).catch(e => console.error("Head fetch error:", e));

        lastHeadUpdate = now;
    }

    // Update Arm (IK / Mapping)
    if (pose && (now - lastArmUpdate > UPDATE_INTERVAL_MS)) {
        const pos = pose.transform.position;

        // Simple mapping
        const baseHeight = 1.0;
        const dy = pos.y - baseHeight;

        let s_lift = Math.max(-60, Math.min(60, dy * 100));
        let s_pan = Math.max(-90, Math.min(90, pos.x * -150));

        const armPayload = {
            positions: {
                shoulder_lift: s_lift,
                shoulder_pan: s_pan
            }
        };

        fetch('/arm', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(armPayload)
        }).catch(e => console.error("Arm fetch error:", e));

        lastArmUpdate = now;
    }
}
