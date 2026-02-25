package org.dynamisphysics.api.event;

import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

public record FootContactEvent(
    Vector3f position,
    Vector3f normal,
    PhysicsMaterial material,
    float contactVelocity
) implements PhysicsEvent {}
