package org.dynamisphysics.api.collision;

import org.dynamisengine.collision.events.CollisionEvent;

/**
 * Physics-owned policy that selects seam-path handling vs legacy fallback handling.
 */
@FunctionalInterface
public interface PhysicsSeamSelectionPolicy<T> {

    boolean usePhysicsSeam(CollisionEvent<T> event);
}
