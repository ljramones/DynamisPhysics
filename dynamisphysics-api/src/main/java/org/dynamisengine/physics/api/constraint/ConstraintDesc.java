package org.dynamisengine.physics.api.constraint;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.vectrix.core.Vector3f;

public record ConstraintDesc(
    ConstraintType type,
    RigidBodyHandle bodyA,
    RigidBodyHandle bodyB,
    Vector3f pivotA,
    Vector3f pivotB,
    Vector3f axisA,
    Vector3f axisB,
    ConstraintLimits limits,
    ConstraintMotor motor,
    float breakForce,
    float breakTorque
) {}
