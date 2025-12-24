
/**
 * RoboCrew WebXR Control Logic
 * Handles VR session, controller tracking, and maps inputs to robot API.
 * Updated to include WebGL layer for proper rendering loop.
 */

let xrSession = null;
let xrReferenceSpace = null;
let animationFrameId = null;
let gl = null; // WebGL Context

// Throttle configuration
const UPDATE_INTERVAL_MS = 100; // Send updates every 100ms
let lastMoveUpdate = 0;
let lastArmUpdate = 0;
let lastHeadUpdate = 0;

// State tracking
const state = {
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
        })
        .catch(err => setStatus("Error checking WebXR support: " + err));
} else {
    setStatus("WebXR API not available (Requires HTTPS or Flag).");
}

function onButtonClicked() {
    if (!xrSession) {
        setStatus("Requesting Session...");
        // Initialize WebGL
        const canvas = document.createElement('canvas');
        gl = canvas.getContext('webgl', { xrCompatible: true });

        navigator.xr.requestSession('immersive-vr', {
            optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking']
        }).then(onSessionStarted)
            .catch(err => {
                setStatus("Session Request Failed: " + err);
                gl = null;
            });
    } else {
        xrSession.end();
    }
}

function onSessionStarted(session) {
    xrSession = session;
    vrButton.textContent = "Exit VR";
    setStatus("VR Session Active - Initializing Layer...");

    session.addEventListener('end', onSessionEnded);

    // Create a WebGL layer (Required for immersive-vr)
    try {
        const glLayer = new XRWebGLLayer(session, gl);
        session.updateRenderState({ baseLayer: glLayer });
    } catch (e) {
        setStatus("Failed to create XRWebGLLayer: " + e);
        return;
    }

    // Get reference space
    // Prefer 'local-floor' for standing/room scale, fallback to 'local'
    session.requestReferenceSpace('local-floor')
        .then((refSpace) => {
            xrReferenceSpace = refSpace;
            setStatus("Reference Space (local-floor) acquired. Starting Loop.");
            animationFrameId = session.requestAnimationFrame(onXRFrame);
        })
        .catch(() => {
            setStatus("local-floor failed, trying local...");
            session.requestReferenceSpace('local')
                .then((refSpace) => {
                    xrReferenceSpace = refSpace;
                    setStatus("Reference Space (local) acquired. Starting Loop.");
                    animationFrameId = session.requestAnimationFrame(onXRFrame);
                })
                .catch(err => setStatus("Failed to get reference space: " + err));
        });
}

function onSessionEnded() {
    xrSession = null;
    vrButton.textContent = "Enter VR";
    setStatus("VR Session Ended");
    gl = null;
}

function onXRFrame(time, frame) {
    const session = frame.session;
    animationFrameId = session.requestAnimationFrame(onXRFrame);

    // Clear the framebuffer (even if we don't draw 3D objects, we must clear)
    // This tells the compositor we are active.
    const glLayer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
    gl.clearColor(0.1, 0.1, 0.1, 1.0); // Dark Gray background
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

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
    const DEADZONE = 0.2; // Slightly larger deadzone
    const finalX = Math.abs(x) > DEADZONE ? x : 0;
    const finalY = Math.abs(y) > DEADZONE ? y : 0;

    const now = Date.now();
    if (now - lastMoveUpdate > UPDATE_INTERVAL_MS) {
        let direction = 'stop';
        if (finalY < -0.4) direction = 'forward';
        else if (finalY > 0.4) direction = 'backward';
        else if (finalX < -0.4) direction = 'left';
        else if (finalX > 0.4) direction = 'right';

        // Send even if stop, to ensure we don't get stuck moving
        // But only send if it changes? No, heartbeat is good.

        fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ direction: direction })
        }).catch(e => console.error(e));

        lastMoveUpdate = now;
    }
}


function handleRightController(source, frame) {
    const gp = source.gamepad;

    // Gripper (Trigger)
    const triggerPressed = gp.buttons[0]?.pressed || false;

    // Head Control (Stick)
    const stickX = gp.axes[2] || 0;
    const stickY = gp.axes[3] || 0;

    // Pose (Arm)
    const pose = frame.getPose(source.gripSpace, xrReferenceSpace);

    const now = Date.now();

    // Check Gripper
    if (triggerPressed !== state.gripper && (now - lastArmUpdate > 500)) {
        state.gripper = triggerPressed;
        fetch('/gripper', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ closed: triggerPressed })
        }).catch(e => console.error(e));
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
        }).catch(e => console.error(e));

        lastHeadUpdate = now;
    }

    // Update Arm
    if (pose && (now - lastArmUpdate > UPDATE_INTERVAL_MS)) {
        const pos = pose.transform.position; // x, y, z (meters)

        // Mapping Logic
        // Quest: Y is Up, Z is Back, X is Right
        // baseHeight: assumed shoulder height ~1.3m standing

        const baseHeight = 1.3;
        const dy = pos.y - baseHeight;

        // Map Y (-0.5m to +0.5m) -> Shoulder Lift (-45 to 45)
        let s_lift = Math.max(-60, Math.min(60, dy * 100));

        // Map X (-0.5m to 0.5m) -> Shoulder Pan (-90 to 90)
        let s_pan = Math.max(-90, Math.min(90, pos.x * -150));

        // Map Z (Depth) - Robot Arm Extension
        // User reaches out (-Z increases). 
        // Arm should extend.
        // Let's assume neutral hand position is ~30cm in front of face.

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
        }).catch(e => console.error(e));

        lastArmUpdate = now;
    }
}
