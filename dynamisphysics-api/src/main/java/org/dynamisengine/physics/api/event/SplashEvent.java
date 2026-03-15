package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.vectrix.core.Vector3f;

public record SplashEvent(
    RigidBodyHandle body,
    Vector3f position,
    Vector3f velocity,
    float submergedVolumeFraction
) implements PhysicsEvent {}
