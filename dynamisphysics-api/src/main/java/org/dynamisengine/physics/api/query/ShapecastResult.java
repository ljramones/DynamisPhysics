package org.dynamisengine.physics.api.query;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.vectrix.core.Vector3f;

public record ShapecastResult(
    RigidBodyHandle body,
    float hitFraction,
    Vector3f normal,
    float penetrationDepth
) {}
