package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.ode4j.ode.DSliderJoint;

/**
 * Deterministic spring controller for SIX_DOF_SPRING constraints.
 * <p>
 * Spring parameters are sourced from {@link ConstraintMotor}:
 * <ul>
 *   <li>targetPosition = linear spring rest position</li>
 *   <li>targetVelocity = linear spring rest velocity</li>
 *   <li>maxForce = linear spring stiffness k (N/m)</li>
 *   <li>maxTorque = linear spring damping c (N*s/m)</li>
 * </ul>
 */
public final class Ode4jSpringController {
    private static final float DEFAULT_STIFFNESS = 40f;
    private static final float DRIVE_VEL = 10f;
    private static final float EPS = 1e-5f;

    private final Ode4jConstraintRegistry constraintRegistry;

    public Ode4jSpringController(Ode4jConstraintRegistry constraintRegistry) {
        this.constraintRegistry = constraintRegistry;
    }

    public void step(float dt) {
        if (!(dt > 0f)) {
            return;
        }
        for (Ode4jConstraintHandle handle : constraintRegistry.constraintsInIdOrder()) {
            if (!handle.isAlive() || handle.type() != ConstraintType.SIX_DOF_SPRING) {
                continue;
            }
            applyLinearSpring(handle.desc().motor(), findSlider(handle));
        }
    }

    private static DSliderJoint findSlider(Ode4jConstraintHandle handle) {
        for (var joint : handle.allJoints()) {
            if (joint instanceof DSliderJoint slider) {
                return slider;
            }
        }
        return null;
    }

    private static void applyLinearSpring(ConstraintMotor motor, DSliderJoint slider) {
        if (slider == null || motor == null || !motor.enabled()) {
            return;
        }

        float k = finitePositive(motor.maxForce(), DEFAULT_STIFFNESS);
        float c = finitePositive(motor.maxTorque(), 2f * (float) java.lang.Math.sqrt(k));
        float restPos = finite(motor.targetPosition(), 0f);
        float restVel = finite(motor.targetVelocity(), 0f);

        float x = (float) slider.getPosition() - restPos;
        float v = (float) slider.getPositionRate() - restVel;
        float force = (-k * x) - (c * v);
        float absForce = java.lang.Math.abs(force);

        if (absForce < EPS) {
            slider.setParamVel(restVel);
            slider.setParamFMax(0f);
            return;
        }

        // Drive direction follows spring force sign; FMax carries magnitude.
        float driveVel = restVel + java.lang.Math.copySign(DRIVE_VEL, force);
        float forceCap = java.lang.Math.min(absForce, k * 8f);
        slider.setParamVel(driveVel);
        slider.setParamFMax(forceCap);
    }

    private static float finite(float value, float fallback) {
        if (Float.isFinite(value)) {
            return value;
        }
        return fallback;
    }

    private static float finitePositive(float value, float fallback) {
        if (Float.isFinite(value) && value > 0f) {
            return value;
        }
        return fallback;
    }
}
