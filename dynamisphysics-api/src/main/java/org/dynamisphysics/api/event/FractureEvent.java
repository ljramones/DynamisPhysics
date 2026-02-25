package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.vectrix.core.Vector3f;

import java.util.List;

public record FractureEvent(
    RigidBodyHandle source,
    Vector3f impactPoint,
    float impactForce,
    List<RigidBodyHandle> fragments
) implements PhysicsEvent {}
