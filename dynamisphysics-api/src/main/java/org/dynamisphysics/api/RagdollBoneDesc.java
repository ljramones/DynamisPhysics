package org.dynamisphysics.api;

import org.dynamiscollision.shapes.CollisionShape;
import org.vectrix.core.Vector3f;

public record RagdollBoneDesc(
    String animisBoneName,
    CollisionShape shape,
    float mass,
    Vector3f localOffset,
    float kp,
    float kd,
    float maxTorque
) {}
