package org.dynamisengine.physics.test.mock;

import org.dynamisengine.physics.api.constraint.ConstraintDesc;
import org.dynamisengine.physics.api.constraint.ConstraintHandle;
import org.dynamisengine.physics.api.constraint.ConstraintType;

public final class MockConstraintHandle implements ConstraintHandle {
    private final ConstraintDesc desc;
    private boolean alive = true;

    public MockConstraintHandle(ConstraintDesc desc) {
        this.desc = desc;
    }

    public void kill() {
        alive = false;
    }

    @Override public boolean isAlive() { return alive; }
    @Override public ConstraintType type() { return desc.type(); }
}
