package org.dynamisphysics.api.collision;

import org.dynamiscollision.events.CollisionEvent;

import java.util.List;

/**
 * Physics-owned ordering policy for seam-path responder strategy execution.
 *
 * <p>This policy governs preferred strategy ordering for fixed-step collision event handling on
 * the Physics seam path.
 */
@FunctionalInterface
public interface PhysicsFixedStepResponderOrderingPolicy<T> {

    List<PhysicsContactResolutionStrategy<T>> orderedStrategies(
            CollisionEvent<T> event,
            List<PhysicsContactResolutionStrategy<T>> baseOrder);
}
