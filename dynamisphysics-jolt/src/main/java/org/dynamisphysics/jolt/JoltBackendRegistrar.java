package org.dynamisphysics.jolt;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.config.PhysicsBackend;

public final class JoltBackendRegistrar {
    static {
        PhysicsWorldFactory.register(PhysicsBackend.JOLT, JoltPhysicsWorld::create);
    }

    public JoltBackendRegistrar() {
    }
}
