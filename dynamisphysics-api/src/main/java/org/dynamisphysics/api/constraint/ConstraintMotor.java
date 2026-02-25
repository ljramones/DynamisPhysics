package org.dynamisphysics.api.constraint;

public record ConstraintMotor(
    boolean enabled,
    float targetVelocity,
    float targetPosition,
    float maxForce,
    float maxTorque
) {
    public static ConstraintMotor off() {
        return new ConstraintMotor(false, 0f, 0f, Float.MAX_VALUE, Float.MAX_VALUE);
    }

    public static ConstraintMotor velocity(float target, float maxForce) {
        return new ConstraintMotor(true, target, 0f, maxForce, Float.MAX_VALUE);
    }

    public static ConstraintMotor position(float target, float maxForce) {
        return new ConstraintMotor(true, 0f, target, maxForce, Float.MAX_VALUE);
    }
}
