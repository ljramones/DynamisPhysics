package org.dynamisphysics.api.collision;

import org.dynamisengine.collision.events.CollisionEventType;
import org.dynamisengine.collision.world.CollisionResponder3D;
import org.dynamisengine.collision.world.CollisionWorld3D;

import java.util.List;

/**
 * Convenience presets for representative Physics-preferred collision flow wiring.
 *
 * <p>Presets are opt-in and additive. Legacy/default behavior remains unchanged unless explicitly
 * configured.
 */
public final class PhysicsCollisionPreferredFlowPresets {

    private PhysicsCollisionPreferredFlowPresets() {
    }

    public static <T> void configureDefault(
            CollisionWorld3D<T> world,
            PhysicsContactBodyAdapter<T> bodyAdapter,
            CollisionResponder3D<T> fallbackResponder) {
        configureDefault(world, bodyAdapter, fallbackResponder, defaultSeamSelectionPolicy());
    }

    public static <T> void configureDefault(
            CollisionWorld3D<T> world,
            PhysicsContactBodyAdapter<T> bodyAdapter,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsSeamSelectionPolicy<T> seamSelectionPolicy) {
        if (world == null) {
            throw new IllegalArgumentException("world must not be null");
        }
        if (bodyAdapter == null) {
            throw new IllegalArgumentException("bodyAdapter must not be null");
        }
        if (seamSelectionPolicy == null) {
            throw new IllegalArgumentException("seamSelectionPolicy must not be null");
        }

        MapBackedWarmStartCachePolicy<T> warmStartPolicy = new MapBackedWarmStartCachePolicy<>();
        PhysicsPreferredCollisionResponder<T> preferred = new PhysicsPreferredCollisionResponder<>(
                List.of(
                        new PositionCorrectionContactResolutionStrategy<>(bodyAdapter),
                        new NormalImpulseContactResolutionStrategy<>(bodyAdapter),
                        new PolicyBackedWarmStartApplicationStrategy<>(bodyAdapter, warmStartPolicy)),
                fallbackResponder,
                warmStartPolicy,
                (contact, loadedBefore) -> loadedBefore,
                (event, base) -> base,
                seamSelectionPolicy);
        PhysicsCollisionPreferredFlowConfigurator.configure(world, preferred);
    }

    private static <T> PhysicsSeamSelectionPolicy<T> defaultSeamSelectionPolicy() {
        return event -> event.responseEnabled()
                && event.type() != CollisionEventType.EXIT
                && event.manifold() != null;
    }
}
