package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;

public record SeparationEvent(
    RigidBodyHandle bodyA,
    RigidBodyHandle bodyB
) implements PhysicsEvent {}
