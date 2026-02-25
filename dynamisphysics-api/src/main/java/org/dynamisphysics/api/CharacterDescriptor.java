package org.dynamisphysics.api;

import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.FootContactHint;

import java.util.function.Consumer;

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
    int collidesWith,
    Consumer<FootContactHint> footIkListener
) {
    public CharacterDescriptor(
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
    ) {
        this(
            height,
            radius,
            mass,
            stepHeight,
            maxSlopeAngleDeg,
            pushForce,
            skinWidth,
            material,
            layer,
            collidesWith,
            null
        );
    }
}
