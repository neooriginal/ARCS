AFRAME.registerComponent('vr-arm-model', {
    schema: {
        scale: { type: 'number', default: 2.0 }
    },

    init: function () {
        this.jointAngles = [0, 0, 0, 0, 0, 0];
        this.buildArm();
    },

    buildArm: function () {
        const s = this.data.scale;
        const linkColor = '#FFD01E';
        const motorColor = '#222';
        const gripperColor = '#444';

        this.base = document.createElement('a-entity');
        this.base.setAttribute('position', '0 0 0');

        const baseBox = document.createElement('a-box');
        baseBox.setAttribute('width', 0.05 * s);
        baseBox.setAttribute('height', 0.02 * s);
        baseBox.setAttribute('depth', 0.05 * s);
        baseBox.setAttribute('color', motorColor);
        this.base.appendChild(baseBox);

        this.shoulder = document.createElement('a-entity');
        this.shoulder.setAttribute('position', `0 ${0.0165 * s} ${-0.0452 * s}`);

        const shoulderCyl = document.createElement('a-cylinder');
        shoulderCyl.setAttribute('height', 0.06 * s);
        shoulderCyl.setAttribute('radius', 0.015 * s);
        shoulderCyl.setAttribute('color', linkColor);
        shoulderCyl.setAttribute('rotation', '90 0 0');
        this.shoulder.appendChild(shoulderCyl);

        this.upperArm = document.createElement('a-entity');
        this.upperArm.setAttribute('position', `0 ${0.0306 * s} ${0.1025 * s}`);

        const upperArmCyl = document.createElement('a-cylinder');
        upperArmCyl.setAttribute('height', 0.10 * s);
        upperArmCyl.setAttribute('radius', 0.012 * s);
        upperArmCyl.setAttribute('color', linkColor);
        upperArmCyl.setAttribute('position', `0 ${0.05 * s} 0`);
        this.upperArm.appendChild(upperArmCyl);

        const upperMotor = document.createElement('a-box');
        upperMotor.setAttribute('width', 0.025 * s);
        upperMotor.setAttribute('height', 0.02 * s);
        upperMotor.setAttribute('depth', 0.018 * s);
        upperMotor.setAttribute('color', motorColor);
        this.upperArm.appendChild(upperMotor);

        this.lowerArm = document.createElement('a-entity');
        this.lowerArm.setAttribute('position', `0 ${0.11257 * s} ${0.028 * s}`);

        const lowerArmCyl = document.createElement('a-cylinder');
        lowerArmCyl.setAttribute('height', 0.12 * s);
        lowerArmCyl.setAttribute('radius', 0.010 * s);
        lowerArmCyl.setAttribute('color', linkColor);
        lowerArmCyl.setAttribute('position', `0 ${0.06 * s} 0`);
        this.lowerArm.appendChild(lowerArmCyl);

        this.wrist = document.createElement('a-entity');
        this.wrist.setAttribute('position', `0 ${0.1349 * s} 0`);

        const wristBox = document.createElement('a-box');
        wristBox.setAttribute('width', 0.025 * s);
        wristBox.setAttribute('height', 0.035 * s);
        wristBox.setAttribute('depth', 0.02 * s);
        wristBox.setAttribute('color', linkColor);
        this.wrist.appendChild(wristBox);

        this.gripper = document.createElement('a-entity');
        this.gripper.setAttribute('position', `0 ${0.0601 * s} 0`);

        const gripperBase = document.createElement('a-box');
        gripperBase.setAttribute('width', 0.03 * s);
        gripperBase.setAttribute('height', 0.02 * s);
        gripperBase.setAttribute('depth', 0.015 * s);
        gripperBase.setAttribute('color', gripperColor);
        this.gripper.appendChild(gripperBase);

        this.fingerLeft = document.createElement('a-box');
        this.fingerLeft.setAttribute('width', 0.005 * s);
        this.fingerLeft.setAttribute('height', 0.04 * s);
        this.fingerLeft.setAttribute('depth', 0.01 * s);
        this.fingerLeft.setAttribute('color', gripperColor);
        this.fingerLeft.setAttribute('position', `${-0.012 * s} ${0.02 * s} 0`);
        this.gripper.appendChild(this.fingerLeft);

        this.fingerRight = document.createElement('a-box');
        this.fingerRight.setAttribute('width', 0.005 * s);
        this.fingerRight.setAttribute('height', 0.04 * s);
        this.fingerRight.setAttribute('depth', 0.01 * s);
        this.fingerRight.setAttribute('color', gripperColor);
        this.fingerRight.setAttribute('position', `${0.012 * s} ${0.02 * s} 0`);
        this.gripper.appendChild(this.fingerRight);

        this.wrist.appendChild(this.gripper);
        this.lowerArm.appendChild(this.wrist);
        this.upperArm.appendChild(this.lowerArm);
        this.shoulder.appendChild(this.upperArm);
        this.base.appendChild(this.shoulder);
        this.el.appendChild(this.base);
    },

    setJointAngles: function (angles) {
        if (!angles || angles.length < 6) return;

        this.jointAngles = angles;
        const s = this.data.scale;

        this.base.setAttribute('rotation', `0 ${-angles[0]} 0`);
        this.shoulder.setAttribute('rotation', `${angles[1]} 0 0`);
        this.upperArm.setAttribute('rotation', `${angles[2]} 0 0`);
        this.lowerArm.setAttribute('rotation', `${angles[3]} 0 0`);
        this.wrist.setAttribute('rotation', `0 ${angles[4]} 0`);

        const gripperAngle = angles[5];
        const openness = Math.max(0, Math.min(1, (gripperAngle + 60) / 140));
        const fingerOffset = 0.008 + openness * 0.01;

        this.fingerLeft.setAttribute('position', `${-fingerOffset * s} ${0.02 * s} 0`);
        this.fingerRight.setAttribute('position', `${fingerOffset * s} ${0.02 * s} 0`);
    }
});
