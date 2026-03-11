package org.dynamisphysics.api.collision;

import org.dynamiscollision.events.CollisionEvent;

/**
 * Physics-owned consumption path for collision contact manifolds.
 *
 * <p>This is an authority seam: Collision provides detection/manifold outputs; Physics consumes
 * and resolves them.
 */
public final class PhysicsContactResolver<T> {

    public void resolve(
            Iterable<DetectedCollisionContact<T>> contacts,
            float deltaSeconds,
            PhysicsContactResolutionStrategy<T> strategy) {
        if (contacts == null) {
            throw new IllegalArgumentException("contacts must not be null");
        }
        if (!Float.isFinite(deltaSeconds) || deltaSeconds <= 0f) {
            throw new IllegalArgumentException("deltaSeconds must be > 0 and finite");
        }
        if (strategy == null) {
            throw new IllegalArgumentException("strategy must not be null");
        }
        for (DetectedCollisionContact<T> contact : contacts) {
            if (contact == null) {
                throw new IllegalArgumentException("contacts must not contain null entries");
            }
            strategy.resolve(contact, deltaSeconds);
        }
    }

    /**
     * Transitional compatibility path: consumes legacy collision events and routes resolution
     * through the Physics-owned seam.
     */
    public void resolveFromCollisionEvents(
            Iterable<CollisionEvent<T>> events,
            float deltaSeconds,
            PhysicsContactResolutionStrategy<T> strategy) {
        resolve(CollisionEventContactAdapter.toDetectedContacts(events), deltaSeconds, strategy);
    }
}
