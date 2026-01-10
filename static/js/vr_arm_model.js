/**
 * VR Arm Model - Displays 3D arm model synced with real robot joint angles
 * Uses primitive geometry (boxes/cylinders) since STL loading is complex in A-Frame
 */

AFRAME.registerComponent('arm-model', {
    schema: {
        scale: { type: 'number', default: 1.0 },
        position: { type: 'vec3', default: { x: 0.5, y: 0.8, z: -0.8 } }
    },

    init: function () {
        this.joints = {};
        this.jointAngles = [0, 0, 0, 0, 0, 0];

        // Materials
        this.yellowMaterial = new THREE.MeshStandardMaterial({
            color: 0xffd21f,
            roughness: 0.6,
            metalness: 0.2
        });
        this.blackMaterial = new THREE.MeshStandardMaterial({
            color: 0x222222,
            roughness: 0.8,
            metalness: 0.1
        });

        // Root container
        this.armRoot = new THREE.Group();
        this.armRoot.position.set(
            this.data.position.x,
            this.data.position.y,
            this.data.position.z
        );
        this.armRoot.scale.setScalar(this.data.scale);
        this.el.object3D.add(this.armRoot);

        this.buildSimpleArm();
        setInterval(() => this.fetchArmPosition(), 200);
        console.log('[ARM MODEL] Initialized');
    },

    buildSimpleArm: function () {
        // Build simplified arm using primitives matching SO100 proportions

        // Base plate
        const baseGeo = new THREE.CylinderGeometry(0.04, 0.05, 0.02, 16);
        const base = new THREE.Mesh(baseGeo, this.yellowMaterial);
        base.position.y = 0.01;
        this.armRoot.add(base);

        // Joint 1: Shoulder Pan (rotates around Y)
        this.joints.shoulder = new THREE.Group();
        this.joints.shoulder.position.set(0, 0.02, 0);
        this.armRoot.add(this.joints.shoulder);

        const shoulderMotor = new THREE.Mesh(
            new THREE.BoxGeometry(0.04, 0.04, 0.04),
            this.blackMaterial
        );
        shoulderMotor.position.y = 0.02;
        this.joints.shoulder.add(shoulderMotor);

        // Joint 2: Shoulder Lift (rotates around X)
        this.joints.upperArm = new THREE.Group();
        this.joints.upperArm.position.set(0, 0.06, 0);
        this.joints.shoulder.add(this.joints.upperArm);

        const upperArm = new THREE.Mesh(
            new THREE.BoxGeometry(0.025, 0.10, 0.025),
            this.yellowMaterial
        );
        upperArm.position.y = 0.05;
        this.joints.upperArm.add(upperArm);

        const elbowMotor = new THREE.Mesh(
            new THREE.BoxGeometry(0.035, 0.03, 0.035),
            this.blackMaterial
        );
        elbowMotor.position.y = 0.10;
        this.joints.upperArm.add(elbowMotor);

        // Joint 3: Elbow Flex (rotates around X)
        this.joints.lowerArm = new THREE.Group();
        this.joints.lowerArm.position.set(0, 0.11, 0);
        this.joints.upperArm.add(this.joints.lowerArm);

        const lowerArm = new THREE.Mesh(
            new THREE.BoxGeometry(0.02, 0.12, 0.02),
            this.yellowMaterial
        );
        lowerArm.position.y = 0.06;
        this.joints.lowerArm.add(lowerArm);

        // Joint 4: Wrist Flex (rotates around X)
        this.joints.wrist = new THREE.Group();
        this.joints.wrist.position.set(0, 0.13, 0);
        this.joints.lowerArm.add(this.joints.wrist);

        const wristPart = new THREE.Mesh(
            new THREE.BoxGeometry(0.03, 0.03, 0.03),
            this.blackMaterial
        );
        this.joints.wrist.add(wristPart);

        // Joint 5: Wrist Roll (rotates around Y)
        this.joints.gripper = new THREE.Group();
        this.joints.gripper.position.set(0, 0.03, 0);
        this.joints.wrist.add(this.joints.gripper);

        // Fixed jaw
        const fixedJaw = new THREE.Mesh(
            new THREE.BoxGeometry(0.008, 0.05, 0.015),
            this.yellowMaterial
        );
        fixedJaw.position.set(-0.012, 0.025, 0);
        this.joints.gripper.add(fixedJaw);

        // Joint 6: Moving jaw
        this.joints.jaw = new THREE.Group();
        this.joints.jaw.position.set(0.012, 0, 0);
        this.joints.gripper.add(this.joints.jaw);

        const movingJaw = new THREE.Mesh(
            new THREE.BoxGeometry(0.008, 0.05, 0.015),
            this.yellowMaterial
        );
        movingJaw.position.y = 0.025;
        this.joints.jaw.add(movingJaw);

        console.log('[ARM MODEL] Arm built with primitives');
    },

    fetchArmPosition: async function () {
        try {
            const res = await fetch('/arm_position');
            if (res.ok) {
                const data = await res.json();
                if (data.positions) {
                    this.jointAngles = [
                        data.positions.shoulder_pan || 0,
                        data.positions.shoulder_lift || 0,
                        data.positions.elbow_flex || 0,
                        data.positions.wrist_flex || 0,
                        data.positions.wrist_roll || 0,
                        data.positions.gripper || 0
                    ];
                }
            }
        } catch (e) {
            // Silently fail - arm might not be connected
        }
    },

    tick: function () {
        if (!this.joints.shoulder) return;

        const deg2rad = Math.PI / 180;

        // Joint 1: Shoulder Pan (Y axis)
        this.joints.shoulder.rotation.y = -this.jointAngles[0] * deg2rad;

        // Joint 2: Shoulder Lift (Z axis - arm rotates forward/back)
        // When 0, arm should point up. Positive = forward
        this.joints.upperArm.rotation.z = this.jointAngles[1] * deg2rad;

        // Joint 3: Elbow Flex (Z axis)
        this.joints.lowerArm.rotation.z = this.jointAngles[2] * deg2rad;

        // Joint 4: Wrist Flex (Z axis, was inverted - fix sign)
        this.joints.wrist.rotation.z = -this.jointAngles[3] * deg2rad;

        // Joint 5: Wrist Roll (Y axis)
        this.joints.gripper.rotation.y = this.jointAngles[4] * deg2rad;

        // Joint 6: Gripper opening (move jaw along X)
        const gripperAngle = this.jointAngles[5] || 0;
        const jawOffset = 0.012 + (gripperAngle / 90) * 0.01;
        this.joints.jaw.position.x = jawOffset;
    },

    remove: function () {
        if (this.armRoot) {
            this.el.object3D.remove(this.armRoot);
        }
    }
});
