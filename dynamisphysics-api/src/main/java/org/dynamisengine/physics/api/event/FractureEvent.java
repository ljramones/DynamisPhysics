package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.vectrix.core.Vector3f;

import java.util.List;

public record FractureEvent(
    RigidBodyHandle source,
    Vector3f impactPoint,
    float impactForce,
    List<RigidBodyHandle> fragments
) implements PhysicsEvent {}
