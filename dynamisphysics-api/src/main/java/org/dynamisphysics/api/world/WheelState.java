package org.dynamisphysics.api.world;

import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

public record WheelState(
    Vector3f contactPosition,
    Vector3f contactNormal,
    float suspensionLength,
    float slipRatio,
    float angularVelocity,
    PhysicsMaterial surfaceMaterial,
    boolean isGrounded
) {
    public static final WheelState ZERO = new WheelState(new Vector3f(), new Vector3f(), 0f, 0f, 0f, PhysicsMaterial.DEFAULT, false);
}
