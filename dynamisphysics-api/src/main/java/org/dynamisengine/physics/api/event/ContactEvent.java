package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.material.PhysicsMaterial;
import org.dynamisengine.vectrix.core.Vector3f;

import java.util.List;

public record ContactEvent(
    RigidBodyHandle bodyA,
    RigidBodyHandle bodyB,
    List<ContactPoint> points,
    float totalImpulse,
    Vector3f relativeVelocity,
    PhysicsMaterial materialA,
    PhysicsMaterial materialB
) implements PhysicsEvent {}
