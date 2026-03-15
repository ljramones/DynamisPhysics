package org.dynamisphysics.api.collision;

import org.dynamisengine.collision.events.CollisionEvent;

/**
 * Physics-owned warm-start cache policy seam.
 *
 * <p>Owns warm-start persistence/invalidation policy for seam-based contact handling.
 */
public interface PhysicsWarmStartCachePolicy<T> {

    PhysicsWarmStartImpulse load(DetectedCollisionContact<T> contact);

    void store(DetectedCollisionContact<T> contact, PhysicsWarmStartImpulse impulse);

    void onCollisionEvent(CollisionEvent<T> event);
}
