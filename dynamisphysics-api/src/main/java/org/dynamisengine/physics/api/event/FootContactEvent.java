package org.dynamisengine.physics.api.event;

import org.dynamisengine.physics.api.material.PhysicsMaterial;
import org.dynamisengine.vectrix.core.Vector3f;

public record FootContactEvent(
    Vector3f position,
    Vector3f normal,
    PhysicsMaterial material,
    float contactVelocity
) implements PhysicsEvent {}
