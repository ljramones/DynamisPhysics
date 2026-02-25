package org.dynamisphysics.api;

import org.dynamisphysics.api.material.PhysicsMaterial;

public record CharacterDescriptor(
    float height,
    float radius,
    float mass,
    float stepHeight,
    float maxSlopeAngleDeg,
    float pushForce,
    float skinWidth,
    PhysicsMaterial material,
    int layer,
    int collidesWith
) {}
