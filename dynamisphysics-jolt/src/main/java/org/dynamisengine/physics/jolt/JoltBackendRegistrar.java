package org.dynamisengine.physics.jolt;

import org.dynamisengine.physics.api.PhysicsWorldFactory;
import org.dynamisengine.physics.api.config.PhysicsBackend;

public final class JoltBackendRegistrar {
    static {
        PhysicsWorldFactory.register(PhysicsBackend.JOLT, JoltPhysicsWorld::create);
    }

    public JoltBackendRegistrar() {
    }
}
