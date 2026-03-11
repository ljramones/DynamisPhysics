package org.dynamisphysics.api.collision;

import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;

import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Default Physics-owned warm-start cache policy using an unordered body-pair key.
 */
public final class MapBackedWarmStartCachePolicy<T> implements PhysicsWarmStartCachePolicy<T> {

    private final Map<PairKey<T>, PhysicsWarmStartImpulse> cache = new ConcurrentHashMap<>();

    @Override
    public PhysicsWarmStartImpulse load(DetectedCollisionContact<T> contact) {
        if (contact == null) {
            throw new IllegalArgumentException("contact must not be null");
        }
        return cache.getOrDefault(PairKey.of(contact.bodyA(), contact.bodyB()), PhysicsWarmStartImpulse.ZERO);
    }

    @Override
    public void store(DetectedCollisionContact<T> contact, PhysicsWarmStartImpulse impulse) {
        if (contact == null) {
            throw new IllegalArgumentException("contact must not be null");
        }
        if (impulse == null) {
            throw new IllegalArgumentException("impulse must not be null");
        }
        cache.put(PairKey.of(contact.bodyA(), contact.bodyB()), impulse);
    }

    @Override
    public void onCollisionEvent(CollisionEvent<T> event) {
        if (event == null) {
            return;
        }
        if (event.type() == CollisionEventType.EXIT) {
            cache.remove(PairKey.of(event.pair().first(), event.pair().second()));
        }
    }

    private record PairKey<T>(T left, T right) {
        private static <T> PairKey<T> of(T a, T b) {
            if (a == null || b == null) {
                throw new IllegalArgumentException("pair values must not be null");
            }
            int ah = System.identityHashCode(a);
            int bh = System.identityHashCode(b);
            if (ah < bh || (ah == bh && System.identityHashCode(a) <= System.identityHashCode(b))) {
                return new PairKey<>(a, b);
            }
            return new PairKey<>(b, a);
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof PairKey<?> other)) {
                return false;
            }
            return Objects.equals(left, other.left) && Objects.equals(right, other.right);
        }

        @Override
        public int hashCode() {
            return Objects.hash(left, right);
        }
    }
}
