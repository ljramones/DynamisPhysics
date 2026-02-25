package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.vectrix.core.Vector3f;

public final class DoorHingeBuilder {
    private DoorHingeBuilder() {}

    public static ConstraintHandle create(
        PhysicsWorld world,
        RigidBodyHandle frame,
        RigidBodyHandle door,
        Vector3f hingePoint,
        Vector3f hingeAxis,
        float minAngleDeg,
        float maxAngleDeg
    ) {
        return world.addConstraint(new ConstraintDesc(
            ConstraintType.HINGE,
            frame,
            door,
            hingePoint,
            hingePoint,
            hingeAxis,
            hingeAxis,
            new ConstraintLimits(
                0f,
                0f,
                (float) Math.toRadians(minAngleDeg),
                (float) Math.toRadians(maxAngleDeg)
            ),
            ConstraintMotor.off(),
            0f,
            0f
        ));
    }

    public static ConstraintHandle createMotorised(
        PhysicsWorld world,
        RigidBodyHandle frame,
        RigidBodyHandle door,
        Vector3f hingePoint,
        Vector3f hingeAxis,
        float minAngleDeg,
        float maxAngleDeg,
        float targetVelocityRadS,
        float maxForce
    ) {
        return world.addConstraint(new ConstraintDesc(
            ConstraintType.HINGE,
            frame,
            door,
            hingePoint,
            hingePoint,
            hingeAxis,
            hingeAxis,
            new ConstraintLimits(
                0f,
                0f,
                (float) Math.toRadians(minAngleDeg),
                (float) Math.toRadians(maxAngleDeg)
            ),
            ConstraintMotor.velocity(targetVelocityRadS, maxForce),
            0f,
            0f
        ));
    }
}
