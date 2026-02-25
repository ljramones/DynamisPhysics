package org.dynamisphysics.api.query;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

public record RaycastResult(
    RigidBodyHandle body,
    Vector3f position,
    Vector3f normal,
    float fraction,
    PhysicsMaterial material,
    int layer
) {}
