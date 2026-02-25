package org.dynamisphysics.api.event;

import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

public record WheelSlipEvent(
    VehicleHandle handle,
    int wheelIndex,
    float slipRatio,
    Vector3f contactPosition,
    PhysicsMaterial surfaceMaterial
) implements PhysicsEvent {}
