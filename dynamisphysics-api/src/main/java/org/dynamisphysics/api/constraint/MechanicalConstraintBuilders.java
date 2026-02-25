package org.dynamisphysics.api.constraint;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.vectrix.core.Vector3f;

/**
 * Builder helpers for mechanical constraints using the 0.2.0 encoding:
 * <ul>
 *   <li>GEAR / RACK_PINION: ratio in {@code limits.linearUpperLimit}</li>
 *   <li>PULLEY: rope length in {@code limits.linearUpperLimit}, ratio in {@code limits.linearLowerLimit}</li>
 * </ul>
 */
public final class MechanicalConstraintBuilders {
    private MechanicalConstraintBuilders() {
    }

    public static ConstraintDesc gear(
        RigidBodyHandle bodyA,
        RigidBodyHandle bodyB,
        Vector3f axisA,
        Vector3f axisB,
        float ratio
    ) {
        return new ConstraintDesc(
            ConstraintType.GEAR,
            bodyA,
            bodyB,
            new Vector3f(),
            new Vector3f(),
            axisA,
            axisB,
            new ConstraintLimits(0f, ratio, 0f, 0f),
            ConstraintMotor.off(),
            0f,
            0f
        );
    }

    public static ConstraintDesc rackPinion(
        RigidBodyHandle rackBody,
        RigidBodyHandle pinionBody,
        Vector3f rackAxis,
        Vector3f pinionAxis,
        float ratio
    ) {
        return new ConstraintDesc(
            ConstraintType.RACK_PINION,
            rackBody,
            pinionBody,
            new Vector3f(),
            new Vector3f(),
            rackAxis,
            pinionAxis,
            new ConstraintLimits(0f, ratio, 0f, 0f),
            ConstraintMotor.off(),
            0f,
            0f
        );
    }

    public static ConstraintDesc pulley(
        RigidBodyHandle bodyA,
        RigidBodyHandle bodyB,
        Vector3f fixedPointA,
        Vector3f fixedPointB,
        Vector3f axisA,
        Vector3f axisB,
        float ropeLength,
        float ratio
    ) {
        return new ConstraintDesc(
            ConstraintType.PULLEY,
            bodyA,
            bodyB,
            fixedPointA,
            fixedPointB,
            axisA,
            axisB,
            new ConstraintLimits(ratio, ropeLength, 0f, 0f),
            ConstraintMotor.off(),
            0f,
            0f
        );
    }
}
