package org.dynamisengine.physics.jolt.body;

import org.dynamisengine.physics.api.body.BodyMode;
import org.dynamisengine.physics.api.body.RigidBodyConfig;
import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.body.StableRigidBodyId;

public final class JoltBodyHandle implements RigidBodyHandle, StableRigidBodyId {
    private final int bodyId;
    private final int joltBodyId;
    private final RigidBodyConfig config;
    private boolean alive = true;

    public JoltBodyHandle(int bodyId, int joltBodyId, RigidBodyConfig config) {
        this.bodyId = bodyId;
        this.joltBodyId = joltBodyId;
        this.config = config;
    }

    public int bodyId() {
        return bodyId;
    }

    public int joltBodyId() {
        return joltBodyId;
    }

    public RigidBodyConfig config() {
        return config;
    }

    public void kill() {
        alive = false;
    }

    @Override
    public boolean isAlive() {
        return alive;
    }

    @Override
    public int layer() {
        return config.layer();
    }

    @Override
    public Object userData() {
        return config.userData();
    }

    @Override
    public BodyMode mode() {
        return config.mode();
    }
}
