package org.dynamisphysics.api.event;

import org.dynamisphysics.api.VehicleHandle;

public record VehicleAirborneEvent(
    VehicleHandle handle,
    boolean isAirborne
) implements PhysicsEvent {}
