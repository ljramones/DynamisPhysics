package org.dynamisphysics.jolt.constraint;

import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.Vec3;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.vectrix.core.Vector3f;

import java.util.List;

public final class JoltMechanicalConstraintController {
    private static final float EPS = 1e-5f;
    private static final float BETA = 0.2f;

    private final BodyInterface bodyInterface;
    private final JoltBodyRegistry bodyRegistry;
    private final JoltConstraintRegistry constraintRegistry;

    public JoltMechanicalConstraintController(
        BodyInterface bodyInterface,
        JoltBodyRegistry bodyRegistry,
        JoltConstraintRegistry constraintRegistry
    ) {
        this.bodyInterface = bodyInterface;
        this.bodyRegistry = bodyRegistry;
        this.constraintRegistry = constraintRegistry;
    }

    public void postSolve(float dt) {
        if (!(dt > 0f)) {
            return;
        }
        List<JoltConstraintHandle> handles = constraintRegistry.constraintsInIdOrder();
        for (JoltConstraintHandle handle : handles) {
            if (!handle.isAlive()) {
                continue;
            }
            switch (handle.type()) {
                case GEAR -> solveGear(handle.desc());
                case RACK_PINION -> solveRackPinion(handle.desc());
                case PULLEY -> solvePulley(handle.desc(), dt);
                default -> {
                }
            }
        }
    }

    private void solveGear(ConstraintDesc desc) {
        JoltBodyHandle a = bodyRegistry.getByHandle(desc.bodyA());
        JoltBodyHandle b = bodyRegistry.getByHandle(desc.bodyB());
        if (!dynamic(a) || !dynamic(b)) {
            return;
        }
        Vector3f axisA = normalized(desc.axisA(), new Vector3f(0f, 1f, 0f));
        Vector3f axisB = normalized(desc.axisB(), new Vector3f(0f, 1f, 0f));
        float ratio = ratioOrDefault(desc.limits().linearUpperLimit(), 1f);
        Vector3f wA3 = toVector3f(bodyInterface.getAngularVelocity(a.joltBodyId()));
        Vector3f wB3 = toVector3f(bodyInterface.getAngularVelocity(b.joltBodyId()));
        float wA = dot(wA3, axisA);
        float wB = dot(wB3, axisB);
        float error = wA + ratio * wB;
        if (java.lang.Math.abs(error) < EPS) {
            return;
        }
        float wA2 = wA - 0.5f * error;
        float wB2 = wB - 0.5f * (error / ratio);
        setAngularComponent(a.joltBodyId(), axisA, wA2);
        setAngularComponent(b.joltBodyId(), axisB, wB2);
    }

    private void solveRackPinion(ConstraintDesc desc) {
        JoltBodyHandle rack = bodyRegistry.getByHandle(desc.bodyA());
        JoltBodyHandle pinion = bodyRegistry.getByHandle(desc.bodyB());
        if (!dynamic(rack) || !dynamic(pinion)) {
            return;
        }
        Vector3f rackAxis = normalized(desc.axisA(), new Vector3f(1f, 0f, 0f));
        Vector3f pinionAxis = normalized(desc.axisB(), new Vector3f(0f, 1f, 0f));
        float ratio = ratioOrDefault(desc.limits().linearUpperLimit(), 1f);
        Vector3f vRack3 = toVector3f(bodyInterface.getLinearVelocity(rack.joltBodyId()));
        Vector3f wPinion3 = toVector3f(bodyInterface.getAngularVelocity(pinion.joltBodyId()));
        float v = dot(vRack3, rackAxis);
        float w = dot(wPinion3, pinionAxis);
        float error = v - ratio * w;
        if (java.lang.Math.abs(error) < EPS) {
            return;
        }
        float v2 = v - 0.5f * error;
        float w2 = w + 0.5f * (error / ratio);
        setLinearComponent(rack.joltBodyId(), rackAxis, v2);
        setAngularComponent(pinion.joltBodyId(), pinionAxis, w2);
    }

    private void solvePulley(ConstraintDesc desc, float dt) {
        JoltBodyHandle a = bodyRegistry.getByHandle(desc.bodyA());
        JoltBodyHandle b = bodyRegistry.getByHandle(desc.bodyB());
        if (!dynamic(a) || !dynamic(b)) {
            return;
        }
        float ratio = ratioOrDefault(desc.limits().linearLowerLimit(), 1f);
        float ropeLength = java.lang.Math.max(0f, desc.limits().linearUpperLimit());
        Vector3f axisA = normalized(desc.axisA(), new Vector3f(0f, 1f, 0f));
        Vector3f axisB = normalized(desc.axisB(), new Vector3f(0f, 1f, 0f));
        Vector3f posA = toVector3f(bodyInterface.getPosition(a.joltBodyId()));
        Vector3f posB = toVector3f(bodyInterface.getPosition(b.joltBodyId()));
        Vector3f anchorA = desc.pivotA();
        Vector3f anchorB = desc.pivotB();
        float dA = dot(posA.sub(anchorA, new Vector3f()), axisA);
        float dB = dot(posB.sub(anchorB, new Vector3f()), axisB);
        float posError = (dA + ratio * dB) - ropeLength;
        float vA = dot(toVector3f(bodyInterface.getLinearVelocity(a.joltBodyId())), axisA);
        float vB = dot(toVector3f(bodyInterface.getLinearVelocity(b.joltBodyId())), axisB);
        float velError = (vA + ratio * vB) + (BETA * posError / dt);
        if (java.lang.Math.abs(velError) < EPS) {
            return;
        }
        float vA2 = vA - 0.5f * velError;
        float vB2 = vB - 0.5f * (velError / ratio);
        setLinearComponent(a.joltBodyId(), axisA, vA2);
        setLinearComponent(b.joltBodyId(), axisB, vB2);
    }

    private void setLinearComponent(int bodyId, Vector3f axis, float target) {
        Vector3f v = toVector3f(bodyInterface.getLinearVelocity(bodyId));
        float current = dot(v, axis);
        Vector3f updated = v.add(axis.mul(target - current, new Vector3f()), new Vector3f());
        bodyInterface.setLinearVelocity(bodyId, new Vec3(updated.x(), updated.y(), updated.z()));
    }

    private void setAngularComponent(int bodyId, Vector3f axis, float target) {
        Vector3f v = toVector3f(bodyInterface.getAngularVelocity(bodyId));
        float current = dot(v, axis);
        Vector3f updated = v.add(axis.mul(target - current, new Vector3f()), new Vector3f());
        bodyInterface.setAngularVelocity(bodyId, new Vec3(updated.x(), updated.y(), updated.z()));
    }

    private static boolean dynamic(JoltBodyHandle handle) {
        return handle != null && handle.isAlive() && handle.mode() == BodyMode.DYNAMIC;
    }

    private static Vector3f normalized(Vector3f axis, Vector3f fallback) {
        if (axis == null) {
            return fallback;
        }
        float len = axis.length();
        if (!(len > EPS)) {
            return fallback;
        }
        return new Vector3f(axis.x() / len, axis.y() / len, axis.z() / len);
    }

    private static float ratioOrDefault(float value, float fallback) {
        if (!Float.isFinite(value) || java.lang.Math.abs(value) < EPS) {
            return fallback;
        }
        return value;
    }

    private static float dot(Vector3f a, Vector3f b) {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    private static Vector3f toVector3f(com.github.stephengold.joltjni.RVec3 v) {
        return new Vector3f((float) v.xx(), (float) v.yy(), (float) v.zz());
    }

    private static Vector3f toVector3f(Vec3 v) {
        return new Vector3f(v.getX(), v.getY(), v.getZ());
    }
}
