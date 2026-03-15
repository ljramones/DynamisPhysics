package org.dynamisengine.physics.ode4j;

import org.dynamisengine.physics.api.PhysicsWorldFactory;
import org.dynamisengine.physics.api.config.PhysicsBackend;

public final class Ode4jBackendRegistrar {
    static {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    public Ode4jBackendRegistrar() {}
}
