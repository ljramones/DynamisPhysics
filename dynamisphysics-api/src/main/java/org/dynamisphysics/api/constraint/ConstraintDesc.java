package org.dynamisphysics.api.constraint;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.vectrix.core.Vector3f;

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
