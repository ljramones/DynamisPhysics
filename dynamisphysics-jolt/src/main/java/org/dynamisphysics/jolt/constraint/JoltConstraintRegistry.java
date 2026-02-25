package org.dynamisphysics.jolt.constraint;

import com.github.stephengold.joltjni.PhysicsSystem;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;

import java.util.LinkedHashMap;
import java.util.Map;

public final class JoltConstraintRegistry {
    private final PhysicsSystem physicsSystem;
    private final JoltBodyRegistry bodyRegistry;
    private final Map<ConstraintHandle, JoltConstraintHandle> byHandle = new LinkedHashMap<>();
    private int nextConstraintId = 1;

    public JoltConstraintRegistry(PhysicsSystem physicsSystem, JoltBodyRegistry bodyRegistry) {
        this.physicsSystem = physicsSystem;
        this.bodyRegistry = bodyRegistry;
    }

    public ConstraintHandle add(ConstraintDesc desc) {
        JoltBodyHandle a = desc.bodyA() == null ? null : bodyRegistry.getByHandle(desc.bodyA());
        JoltBodyHandle b = desc.bodyB() == null ? null : bodyRegistry.getByHandle(desc.bodyB());
        if (a == null || b == null) {
            throw new IllegalArgumentException("Both constraint bodies must be live Jolt bodies");
        }
        JoltConstraintHandle handle = JoltConstraintFactory.create(nextConstraintId++, desc, physicsSystem, a, b);
        physicsSystem.addConstraint(handle.constraint());
        byHandle.put(handle, handle);
        return handle;
    }

    public void remove(ConstraintHandle handle) {
        JoltConstraintHandle jh = byHandle.remove(handle);
        if (jh == null) {
            return;
        }
        physicsSystem.removeConstraint(jh.constraint());
        jh.kill();
    }

    public void setEnabled(ConstraintHandle handle, boolean enabled) {
        JoltConstraintHandle jh = byHandle.get(handle);
        if (jh != null) {
            jh.constraint().setEnabled(enabled);
        }
    }

    public void setMotorTarget(ConstraintHandle handle, float targetVelocityOrPosition) {
        JoltConstraintHandle jh = byHandle.get(handle);
        if (jh != null) {
            JoltConstraintFactory.setMotorTarget(jh, targetVelocityOrPosition);
        }
    }

    public void clearAll() {
        for (ConstraintHandle handle : java.util.List.copyOf(byHandle.keySet())) {
            remove(handle);
        }
    }
}
