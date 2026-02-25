package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;

public record WakeEvent(RigidBodyHandle body) implements PhysicsEvent {}
