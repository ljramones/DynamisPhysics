package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;

public record TriggerEnterEvent(
    RigidBodyHandle trigger,
    RigidBodyHandle body
) implements PhysicsEvent {}
