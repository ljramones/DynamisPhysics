package org.dynamisphysics.ode4j;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.config.PhysicsBackend;

public final class Ode4jBackendRegistrar {
    static {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    public Ode4jBackendRegistrar() {}
}
