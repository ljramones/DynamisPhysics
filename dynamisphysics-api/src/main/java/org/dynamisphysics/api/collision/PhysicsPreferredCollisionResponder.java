package org.dynamisphysics.api.collision;

import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.world.CollisionResponder3D;

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

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder) {
        this(preferredStrategies, fallbackResponder, null);
    }

    public PhysicsPreferredCollisionResponder(
            List<PhysicsContactResolutionStrategy<T>> preferredStrategies,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsWarmStartCachePolicy<T> warmStartCachePolicy) {
        if (preferredStrategies == null) {
            throw new IllegalArgumentException("preferredStrategies must not be null");
        }
        this.resolver = new PhysicsContactResolver<>();
        this.preferredStrategies = List.copyOf(preferredStrategies);
        this.fallbackResponder = fallbackResponder;
        this.warmStartCachePolicy = warmStartCachePolicy;
    }

    @Override
    public void resolve(CollisionEvent<T> event) {
        if (event == null) {
            return;
        }
        if (warmStartCachePolicy != null) {
            warmStartCachePolicy.onCollisionEvent(event);
        }
        if (canResolveViaPhysicsSeam(event) && !preferredStrategies.isEmpty()) {
            DetectedCollisionContact<T> contact = new DetectedCollisionContact<>(
                    event.pair().first(),
                    event.pair().second(),
                    event.manifold());
            resolver.resolve(
                    List.of(contact),
                    1f / 60f,
                    (c, dt) -> {
                        for (PhysicsContactResolutionStrategy<T> strategy : preferredStrategies) {
                            strategy.resolve(c, dt);
                        }
                    });
            return;
        }
        if (fallbackResponder != null) {
            fallbackResponder.resolve(event);
        }
    }

    private static <T> boolean canResolveViaPhysicsSeam(CollisionEvent<T> event) {
        return event.responseEnabled()
                && event.type() != CollisionEventType.EXIT
                && event.manifold() != null;
    }
}
