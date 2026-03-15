package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;

public record TriggerExitEvent(
    RigidBodyHandle trigger,
    RigidBodyHandle body
) implements PhysicsEvent {}
