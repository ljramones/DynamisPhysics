package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;

public record SeparationEvent(
    RigidBodyHandle bodyA,
    RigidBodyHandle bodyB
) implements PhysicsEvent {}
