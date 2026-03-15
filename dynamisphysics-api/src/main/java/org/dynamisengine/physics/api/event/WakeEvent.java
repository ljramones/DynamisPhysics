package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;

public record WakeEvent(RigidBodyHandle body) implements PhysicsEvent {}
