package org.dynamisengine.physics.api.collision;

import org.dynamisengine.collision.events.CollisionEvent;
import org.dynamisengine.collision.events.CollisionEventType;
import org.dynamisengine.collision.world.CollisionResponder3D;

import java.util.List;

/**
 * Physics-owned orchestration bridge for collision step paths.
 *
 * <p>Prefers Physics seam resolution strategies for applicable contact events, and falls back to a
 * legacy collision responder only when the seam does not apply.
 */
public final class PhysicsPreferredCollisionResponder<T> implements CollisionResponder3D<T> {

    private final PhysicsContactResolver<T> resolver;
    private final List<PhysicsContactResolutionStrategy<T>> preferredStrategies;
    private final CollisionResponder3D<T> fallbackResponder;
    private final PhysicsWarmStartCachePolicy<T> warmStartCachePolicy;
    private final PhysicsWarmStartPersistenceStrategy<T> warmStartPersistenceStrategy;
    private final PhysicsFixedStepResponderOrderingPolicy<T> orderingPolicy;
    private final PhysicsSeamSelectionPolicy<T> seamSelectionPolicy;

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder) {
        this(
                preferredStrategies,
                fallbackResponder,
                null,
                (contact, loaded) -> loaded,
                (event, base) -> base,
                PhysicsPreferredCollisionResponder::defaultSeamSelection);
    }

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsWarmStartCachePolicy<T> warmStartCachePolicy) {
        this(
                preferredStrategies,
                fallbackResponder,
                warmStartCachePolicy,
                (contact, loaded) -> loaded,
                (event, base) -> base,
                PhysicsPreferredCollisionResponder::defaultSeamSelection);
    }

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsWarmStartCachePolicy<T> warmStartCachePolicy,
            PhysicsWarmStartPersistenceStrategy<T> warmStartPersistenceStrategy) {
        this(
                preferredStrategies,
                fallbackResponder,
                warmStartCachePolicy,
                warmStartPersistenceStrategy,
                (event, base) -> base,
                PhysicsPreferredCollisionResponder::defaultSeamSelection);
    }

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsWarmStartCachePolicy<T> warmStartCachePolicy,
            PhysicsWarmStartPersistenceStrategy<T> warmStartPersistenceStrategy,
            PhysicsFixedStepResponderOrderingPolicy<T> orderingPolicy) {
        this(
                preferredStrategies,
                fallbackResponder,
                warmStartCachePolicy,
                warmStartPersistenceStrategy,
                orderingPolicy,
                PhysicsPreferredCollisionResponder::defaultSeamSelection);
    }

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsWarmStartCachePolicy<T> warmStartCachePolicy,
            PhysicsWarmStartPersistenceStrategy<T> warmStartPersistenceStrategy,
            PhysicsFixedStepResponderOrderingPolicy<T> orderingPolicy,
            PhysicsSeamSelectionPolicy<T> seamSelectionPolicy) {
        if (preferredStrategies == null) {
            throw new IllegalArgumentException("preferredStrategies must not be null");
        }
        if (warmStartCachePolicy != null && warmStartPersistenceStrategy == null) {
            throw new IllegalArgumentException("warmStartPersistenceStrategy must not be null when warmStartCachePolicy is set");
        }
        if (orderingPolicy == null) {
            throw new IllegalArgumentException("orderingPolicy must not be null");
        }
        if (seamSelectionPolicy == null) {
            throw new IllegalArgumentException("seamSelectionPolicy must not be null");
        }
        this.resolver = new PhysicsContactResolver<>();
        this.preferredStrategies = List.copyOf(preferredStrategies);
        this.fallbackResponder = fallbackResponder;
        this.warmStartCachePolicy = warmStartCachePolicy;
        this.warmStartPersistenceStrategy = warmStartPersistenceStrategy;
        this.orderingPolicy = orderingPolicy;
        this.seamSelectionPolicy = seamSelectionPolicy;
    }

    @Override
    public void resolve(CollisionEvent<T> event) {
        if (event == null) {
            return;
        }
        if (warmStartCachePolicy != null) {
            warmStartCachePolicy.onCollisionEvent(event);
        }
        if (seamSelectionPolicy.usePhysicsSeam(event) && !preferredStrategies.isEmpty()) {
            DetectedCollisionContact<T> contact = new DetectedCollisionContact<>(
                    event.pair().first(),
                    event.pair().second(),
                    event.manifold());
            PhysicsWarmStartImpulse loadedBefore = warmStartCachePolicy == null
                    ? PhysicsWarmStartImpulse.ZERO
                    : warmStartCachePolicy.load(contact);
            resolver.resolve(
                    List.of(contact),
                    1f / 60f,
                    (c, dt) -> {
                        List<PhysicsContactResolutionStrategy<T>> ordered = orderingPolicy.orderedStrategies(event, preferredStrategies);
                        if (ordered == null) {
                            throw new IllegalStateException("orderingPolicy must return a strategy list");
                        }
                        for (PhysicsContactResolutionStrategy<T> strategy : ordered) {
                            strategy.resolve(c, dt);
                        }
                    });
            if (warmStartCachePolicy != null) {
                PhysicsWarmStartImpulse updated = warmStartPersistenceStrategy.afterResolution(contact, loadedBefore);
                warmStartCachePolicy.store(contact, updated == null ? PhysicsWarmStartImpulse.ZERO : updated);
            }
            return;
        }
        if (fallbackResponder != null) {
            fallbackResponder.resolve(event);
        }
    }

    private static <T> boolean defaultSeamSelection(CollisionEvent<T> event) {
        return event.responseEnabled()
                && event.type() != CollisionEventType.EXIT
                && event.manifold() != null;
    }
}
