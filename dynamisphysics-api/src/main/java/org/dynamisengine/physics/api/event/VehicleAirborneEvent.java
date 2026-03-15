package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.VehicleHandle;

public record VehicleAirborneEvent(
    VehicleHandle handle,
    boolean isAirborne
) implements PhysicsEvent {}
