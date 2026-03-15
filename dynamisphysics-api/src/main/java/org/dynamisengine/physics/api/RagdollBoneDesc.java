package org.dynamisengine.physics.api;

import org.dynamisengine.collision.shapes.CollisionShape;
import org.dynamisengine.vectrix.core.Vector3f;

public record RagdollBoneDesc(
    String animisBoneName,
    CollisionShape shape,
    float mass,
    Vector3f localOffset,
    float kp,
    float kd,
    float maxTorque
) {}
