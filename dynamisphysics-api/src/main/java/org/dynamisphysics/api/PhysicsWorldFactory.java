package org.dynamisphysics.api;

import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;

import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

/**
 * Backend modules register world constructors at startup.
 */
public final class PhysicsWorldFactory {
    private static final Map<PhysicsBackend, Function<PhysicsWorldConfig, PhysicsWorld>> REGISTRY =
        new EnumMap<>(PhysicsBackend.class);

    private PhysicsWorldFactory() {}

    public static void register(PhysicsBackend backend, Function<PhysicsWorldConfig, PhysicsWorld> constructor) {
        REGISTRY.put(Objects.requireNonNull(backend, "backend"), Objects.requireNonNull(constructor, "constructor"));
    }

    public static PhysicsWorld create(PhysicsWorldConfig config) {
        Objects.requireNonNull(config, "config");
        Function<PhysicsWorldConfig, PhysicsWorld> constructor = REGISTRY.get(config.backend());
        if (constructor == null) {
            throw new IllegalStateException("No physics backend registered for " + config.backend());
        }
        return constructor.apply(config);
    }
}
