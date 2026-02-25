package org.dynamisphysics.api.world;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

public record CharacterState(
    Vector3f position,
    Vector3f velocity,
    boolean isGrounded,
    PhysicsMaterial groundMaterial,
    Vector3f groundNormal,
    RigidBodyHandle groundBody,
    Vector3f inheritedVelocity
) {
    public static final CharacterState ZERO = new CharacterState(
        new Vector3f(),
        new Vector3f(),
        false,
        PhysicsMaterial.DEFAULT,
        new Vector3f(0f, 1f, 0f),
        null,
        new Vector3f()
    );
}
