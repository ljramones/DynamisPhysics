package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;

public record SleepEvent(RigidBodyHandle body) implements PhysicsEvent {}
