package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;

public record TriggerExitEvent(
    RigidBodyHandle trigger,
    RigidBodyHandle body
) implements PhysicsEvent {}
