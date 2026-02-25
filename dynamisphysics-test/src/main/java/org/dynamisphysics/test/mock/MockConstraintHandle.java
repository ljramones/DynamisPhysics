package org.dynamisphysics.test.mock;

import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintType;

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
