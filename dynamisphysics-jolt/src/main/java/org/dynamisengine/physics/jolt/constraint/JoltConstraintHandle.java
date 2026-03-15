package org.dynamisengine.physics.jolt.constraint;

import com.github.stephengold.joltjni.Constraint;
import org.dynamisengine.physics.api.constraint.ConstraintDesc;
import org.dynamisengine.physics.api.constraint.ConstraintHandle;
import org.dynamisengine.physics.api.constraint.ConstraintType;

public final class JoltConstraintHandle implements ConstraintHandle {
    private final int constraintId;
    private final ConstraintDesc desc;
    private final Constraint constraint;
    private boolean alive = true;

    public JoltConstraintHandle(int constraintId, ConstraintDesc desc, Constraint constraint) {
        this.constraintId = constraintId;
        this.desc = desc;
        this.constraint = constraint;
    }

    public int constraintId() {
        return constraintId;
    }

    public ConstraintDesc desc() {
        return desc;
    }

    public Constraint constraint() {
        return constraint;
    }

    public boolean hasNativeConstraint() {
        return constraint != null;
    }

    public void kill() {
        this.alive = false;
    }

    @Override
    public boolean isAlive() {
        return alive;
    }

    @Override
    public ConstraintType type() {
        return desc.type();
    }
}
