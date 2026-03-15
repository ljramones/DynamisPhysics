package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.constraint.ConstraintHandle;

public record ConstraintBreakEvent(
    ConstraintHandle constraint,
    float breakForce
) implements PhysicsEvent {}
