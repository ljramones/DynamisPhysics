package org.dynamisphysics.api.query;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.vectrix.core.Vector3f;

public record ShapecastResult(
    RigidBodyHandle body,
    float hitFraction,
    Vector3f normal,
    float penetrationDepth
) {}
