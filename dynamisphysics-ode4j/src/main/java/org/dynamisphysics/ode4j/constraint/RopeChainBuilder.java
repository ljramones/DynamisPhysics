package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class RopeChainBuilder {
    private RopeChainBuilder() {}

    public static List<ConstraintHandle> create(PhysicsWorld world, List<RigidBodyHandle> links) {
        if (links.size() < 2) {
            throw new IllegalArgumentException("Rope requires at least 2 links, got: " + links.size());
        }
        var handles = new ArrayList<ConstraintHandle>(links.size() - 1);
        for (int i = 0; i < links.size() - 1; i++) {
            handles.add(world.addConstraint(new ConstraintDesc(
                ConstraintType.BALL_SOCKET,
                links.get(i),
                links.get(i + 1),
                new Vector3f(),
                new Vector3f(),
                new Vector3f(0f, 1f, 0f),
                new Vector3f(0f, 1f, 0f),
                ConstraintLimits.free(),
                ConstraintMotor.off(),
                0f,
                0f
            )));
        }
        return Collections.unmodifiableList(handles);
    }

    public static List<ConstraintHandle> createBreakable(
        PhysicsWorld world,
        List<RigidBodyHandle> links,
        float breakForceN
    ) {
        if (links.size() < 2) {
            throw new IllegalArgumentException("Rope requires at least 2 links, got: " + links.size());
        }
        var handles = new ArrayList<ConstraintHandle>(links.size() - 1);
        for (int i = 0; i < links.size() - 1; i++) {
            handles.add(world.addConstraint(new ConstraintDesc(
                ConstraintType.BALL_SOCKET,
                links.get(i),
                links.get(i + 1),
                new Vector3f(),
                new Vector3f(),
                new Vector3f(0f, 1f, 0f),
                new Vector3f(0f, 1f, 0f),
                ConstraintLimits.free(),
                ConstraintMotor.off(),
                breakForceN,
                0f
            )));
        }
        return Collections.unmodifiableList(handles);
    }
}
