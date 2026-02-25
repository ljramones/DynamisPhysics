package org.dynamisphysics.api.constraint;

public record ConstraintLimits(
    float linearLowerLimit,
    float linearUpperLimit,
    float angularLowerLimit,
    float angularUpperLimit
) {
    public static ConstraintLimits free() {
        return new ConstraintLimits(-Float.MAX_VALUE, Float.MAX_VALUE, -Float.MAX_VALUE, Float.MAX_VALUE);
    }

    public static ConstraintLimits locked() {
        return new ConstraintLimits(0f, 0f, 0f, 0f);
    }
}
