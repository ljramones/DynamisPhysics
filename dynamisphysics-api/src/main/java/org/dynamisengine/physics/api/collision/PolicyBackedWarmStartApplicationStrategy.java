package org.dynamisengine.physics.api.collision;

/**
 * Physics-owned warm-start application strategy backed by Physics cache policy.
 */
public final class PolicyBackedWarmStartApplicationStrategy<T> implements PhysicsContactResolutionStrategy<T> {

    private final WarmStartApplicationContactResolutionStrategy<T> delegate;

    public PolicyBackedWarmStartApplicationStrategy(
            PhysicsContactBodyAdapter<T> bodyAdapter,
            PhysicsWarmStartCachePolicy<T> cachePolicy) {
        if (bodyAdapter == null) {
            throw new IllegalArgumentException("bodyAdapter must not be null");
        }
        if (cachePolicy == null) {
            throw new IllegalArgumentException("cachePolicy must not be null");
        }
        this.delegate = new WarmStartApplicationContactResolutionStrategy<>(bodyAdapter, cachePolicy::load);
    }

    @Override
    public void resolve(DetectedCollisionContact<T> contact, float deltaSeconds) {
        delegate.resolve(contact, deltaSeconds);
    }
}
