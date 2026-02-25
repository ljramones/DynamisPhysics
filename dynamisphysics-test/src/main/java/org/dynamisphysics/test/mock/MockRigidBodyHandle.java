package org.dynamisphysics.test.mock;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;

public final class MockRigidBodyHandle implements RigidBodyHandle {
    private final RigidBodyConfig config;
    private boolean alive = true;

    public MockRigidBodyHandle(RigidBodyConfig config) {
        this.config = config;
    }

    public void kill() {
        alive = false;
    }

    @Override public boolean isAlive() { return alive; }
    @Override public int layer() { return config.layer(); }
    @Override public Object userData() { return config.userData(); }
    @Override public BodyMode mode() { return config.mode(); }
}
