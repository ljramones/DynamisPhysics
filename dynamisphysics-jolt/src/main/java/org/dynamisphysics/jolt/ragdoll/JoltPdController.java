package org.dynamisphysics.jolt.ragdoll;

import org.vectrix.core.Vector3f;
import org.vectrix.physics.PdControllersf;

final class JoltPdController {
    private JoltPdController() {
    }

    static Vector3f computeTorque(
        Vector3f angularError,
        float kp,
        Vector3f angularVelocity,
        float kd,
        float maxTorque,
        float alpha
    ) {
        Vector3f torque = PdControllersf.torque(angularError, kp, angularVelocity, kd, new Vector3f());
        torque.mul(alpha);
        return PdControllersf.clampTorque(torque, maxTorque, new Vector3f());
    }
}

