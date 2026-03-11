package org.dynamisphysics.api.collision;

import org.dynamiscollision.world.CollisionWorld3D;

/**
 * Production-facing representative entrypoint for configuring Physics-preferred collision response flow.
 *
 * <p>This is opt-in wiring. Legacy/default behavior remains unchanged unless this configurator is
 * applied by the caller.
 */
public final class PhysicsCollisionPreferredFlowConfigurator {

    private PhysicsCollisionPreferredFlowConfigurator() {
    }

    public static <T> void configure(
            CollisionWorld3D<T> world,
            PhysicsPreferredCollisionResponder<T> preferredResponder) {
        if (world == null) {
            throw new IllegalArgumentException("world must not be null");
        }
        if (preferredResponder == null) {
            throw new IllegalArgumentException("preferredResponder must not be null");
        }
        // Representative preferred-path wiring: force explicit responder handling path in CollisionWorld.
        world.setResponsePathPolicy((candidateResponder, responseEvents) -> false);
        world.setResponder(preferredResponder);
    }
}
