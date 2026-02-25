package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.ode4j.ode.DJoint;

import java.util.ArrayList;
import java.util.List;

public final class Ode4jConstraintHandle implements ConstraintHandle {
    private final ConstraintDesc desc;
    private final DJoint joint;
    private final List<DJoint> joints;
    private boolean alive = true;

    public Ode4jConstraintHandle(ConstraintDesc desc, DJoint joint) {
        this.desc = desc;
        this.joint = joint;
        this.joints = joint != null ? List.of(joint) : List.of();
    }

    public Ode4jConstraintHandle(ConstraintDesc desc, List<DJoint> joints) {
        this.desc = desc;
        this.joint = joints.isEmpty() ? null : joints.get(0);
        this.joints = List.copyOf(joints);
    }

    public DJoint primaryJoint() { return joint; }
    public List<DJoint> allJoints() { return joints; }
    public ConstraintDesc desc() { return desc; }

    public void kill() {
        alive = false;
        for (DJoint j : new ArrayList<>(joints)) {
            try {
                j.destroy();
            } catch (Exception ignored) {
                // Constraint cleanup should be best-effort.
            }
        }
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
