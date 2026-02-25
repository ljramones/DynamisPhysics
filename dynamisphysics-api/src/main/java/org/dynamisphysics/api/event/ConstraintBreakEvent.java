package org.dynamisphysics.api.event;

import org.dynamisphysics.api.constraint.ConstraintHandle;

public record ConstraintBreakEvent(
    ConstraintHandle constraint,
    float breakForce
) implements PhysicsEvent {}
