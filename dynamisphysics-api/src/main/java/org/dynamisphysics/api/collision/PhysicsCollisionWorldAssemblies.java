package org.dynamisphysics.api.collision;

import org.dynamisengine.collision.bounds.Aabb;
import org.dynamisengine.collision.broadphase.BroadPhase3D;
import org.dynamisengine.collision.contact.ContactManifold3D;
import org.dynamisengine.collision.filtering.CollisionFilter;
import org.dynamisengine.collision.world.CollisionResponder3D;
import org.dynamisengine.collision.world.CollisionWorld3D;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;

/**
 * Production-facing runtime composition helpers for collision world + Physics preferred flow wiring.
 *
 * <p>These assemblies are opt-in and additive. Legacy/default behavior remains unchanged for callsites
 * that do not use them.
 */
public final class PhysicsCollisionWorldAssemblies {

    private PhysicsCollisionWorldAssemblies() {
    }

    public static <T> CollisionWorld3D<T> createWithPreferredDefaults(
            BroadPhase3D<T> broadPhase,
            Function<T, Aabb> boundsProvider,
            Function<T, CollisionFilter> filterProvider,
            BiFunction<T, T, Optional<ContactManifold3D>> narrowPhase,
            PhysicsContactBodyAdapter<T> bodyAdapter,
            CollisionResponder3D<T> fallbackResponder) {
        CollisionWorld3D<T> world = new CollisionWorld3D<>(
                broadPhase,
                boundsProvider,
                filterProvider,
                narrowPhase);
        PhysicsCollisionPreferredFlowConfigurator.configureDefault(world, bodyAdapter, fallbackResponder);
        return world;
    }

    public static <T> CollisionWorld3D<T> createWithPreferredDefaults(
            BroadPhase3D<T> broadPhase,
            Function<T, Aabb> boundsProvider,
            Function<T, CollisionFilter> filterProvider,
            BiFunction<T, T, Optional<ContactManifold3D>> narrowPhase,
            PhysicsContactBodyAdapter<T> bodyAdapter,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsSeamSelectionPolicy<T> seamSelectionPolicy) {
        CollisionWorld3D<T> world = new CollisionWorld3D<>(
                broadPhase,
                boundsProvider,
                filterProvider,
                narrowPhase);
        PhysicsCollisionPreferredFlowConfigurator.configureDefault(
                world,
                bodyAdapter,
                fallbackResponder,
                seamSelectionPolicy);
        return world;
    }
}
