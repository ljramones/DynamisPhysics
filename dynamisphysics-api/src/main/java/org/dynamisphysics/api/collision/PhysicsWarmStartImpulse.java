package org.dynamisphysics.api.collision;

/**
 * Physics-owned warm-start impulse value.
 */
public record PhysicsWarmStartImpulse(
        double normalImpulse,
        double tangentImpulse) {

    public static final PhysicsWarmStartImpulse ZERO = new PhysicsWarmStartImpulse(0.0, 0.0);
}
