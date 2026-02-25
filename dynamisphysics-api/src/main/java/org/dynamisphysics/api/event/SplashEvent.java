package org.dynamisphysics.api.event;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.vectrix.core.Vector3f;

public record SplashEvent(
    RigidBodyHandle body,
    Vector3f position,
    Vector3f velocity,
    float submergedVolumeFraction
) implements PhysicsEvent {}
