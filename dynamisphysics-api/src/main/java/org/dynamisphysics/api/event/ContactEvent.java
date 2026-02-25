package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

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
