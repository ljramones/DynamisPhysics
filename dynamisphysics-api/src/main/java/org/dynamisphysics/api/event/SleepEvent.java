package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;

public record SleepEvent(RigidBodyHandle body) implements PhysicsEvent {}
