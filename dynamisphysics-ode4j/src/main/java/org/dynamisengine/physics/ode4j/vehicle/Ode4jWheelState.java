package org.dynamisengine.physics.ode4j.vehicle;

import org.dynamisengine.physics.api.material.PhysicsMaterial;
import org.dynamisengine.physics.api.world.WheelState;
import org.dynamisengine.vectrix.core.Vector3f;

final class Ode4jWheelState {
    Vector3f contactPosition = new Vector3f();
    Vector3f contactNormal = new Vector3f();
    float suspensionLength = 0f;
    float slipRatio = 0f;
    float angularVelocity = 0f;
    PhysicsMaterial surfaceMaterial = PhysicsMaterial.DEFAULT;
    boolean grounded = false;

    WheelState toPublic() {
        return new WheelState(
            new Vector3f(contactPosition),
            new Vector3f(contactNormal),
            suspensionLength,
            slipRatio,
            angularVelocity,
            surfaceMaterial,
            grounded
        );
    }
}
