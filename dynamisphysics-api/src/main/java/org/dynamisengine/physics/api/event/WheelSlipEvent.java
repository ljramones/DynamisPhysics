package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.VehicleHandle;
import org.dynamisengine.physics.api.material.PhysicsMaterial;
import org.dynamisengine.vectrix.core.Vector3f;

public record WheelSlipEvent(
    VehicleHandle handle,
    int wheelIndex,
    float slipRatio,
    Vector3f contactPosition,
    PhysicsMaterial surfaceMaterial
) implements PhysicsEvent {}
