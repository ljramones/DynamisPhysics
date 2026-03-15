package org.dynamisengine.physics.api.collision;

/**
 * Physics-owned contact-resolution strategy for collision detection outputs.
 *
 * <p>Implementations apply simulation policy (impulses/constraints/response) in Physics,
 * not in Collision substrate code.
 */
@FunctionalInterface
public interface PhysicsContactResolutionStrategy<T> {

    void resolve(DetectedCollisionContact<T> contact, float deltaSeconds);
}
