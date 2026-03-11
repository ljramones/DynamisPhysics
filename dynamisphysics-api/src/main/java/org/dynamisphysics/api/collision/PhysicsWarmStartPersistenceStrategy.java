package org.dynamisphysics.api.collision;

/**
 * Physics-owned policy for post-resolution warm-start persistence updates.
 */
@FunctionalInterface
public interface PhysicsWarmStartPersistenceStrategy<T> {

    PhysicsWarmStartImpulse afterResolution(
            DetectedCollisionContact<T> contact,
            PhysicsWarmStartImpulse loadedBeforeResolution);
}
