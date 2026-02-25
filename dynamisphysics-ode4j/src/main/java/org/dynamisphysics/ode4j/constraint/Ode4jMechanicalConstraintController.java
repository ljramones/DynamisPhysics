package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.vectrix.core.Vector3f;

public final class Ode4jMechanicalConstraintController {
    private static final float EPS = 1e-5f;
    private static final float BETA = 0.2f;

    private final Ode4jConstraintRegistry constraintRegistry;
    private final Ode4jBodyRegistry bodyRegistry;

    public Ode4jMechanicalConstraintController(
        Ode4jConstraintRegistry constraintRegistry,
        Ode4jBodyRegistry bodyRegistry
    ) {
        this.constraintRegistry = constraintRegistry;
        this.bodyRegistry = bodyRegistry;
    }

    public void postSolve(float dt) {
        if (!(dt > 0f)) {
            return;
        }
        for (Ode4jConstraintHandle handle : constraintRegistry.constraintsInIdOrder()) {
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
        Ode4jBodyHandle a = bodyRegistry.getHandle(desc.bodyA());
        Ode4jBodyHandle b = bodyRegistry.getHandle(desc.bodyB());
        if (!dynamic(a) || !dynamic(b)) {
            return;
        }
        Vector3f axisA = normalized(desc.axisA(), new Vector3f(0f, 1f, 0f));
        Vector3f axisB = normalized(desc.axisB(), new Vector3f(0f, 1f, 0f));
        float ratio = ratioOrDefault(desc.limits().linearUpperLimit(), 1f);

        float wA = dot(toVec(a.body().getAngularVel()), axisA);
        float wB = dot(toVec(b.body().getAngularVel()), axisB);
        float error = wA + (ratio * wB);
        if (java.lang.Math.abs(error) < EPS) {
            return;
        }
        float wA2 = wA - 0.5f * error;
        float wB2 = wB - 0.5f * (error / ratio);
        setAngularComponent(a.body(), axisA, wA2);
        setAngularComponent(b.body(), axisB, wB2);
    }

    private void solveRackPinion(ConstraintDesc desc) {
        Ode4jBodyHandle rack = bodyRegistry.getHandle(desc.bodyA());
        Ode4jBodyHandle pinion = bodyRegistry.getHandle(desc.bodyB());
        if (!dynamic(rack) || !dynamic(pinion)) {
            return;
        }
        Vector3f rackAxis = normalized(desc.axisA(), new Vector3f(1f, 0f, 0f));
        Vector3f pinionAxis = normalized(desc.axisB(), new Vector3f(0f, 1f, 0f));
        float ratio = ratioOrDefault(desc.limits().linearUpperLimit(), 1f);

        float v = dot(toVec(rack.body().getLinearVel()), rackAxis);
        float w = dot(toVec(pinion.body().getAngularVel()), pinionAxis);
        float error = v - (ratio * w);
        if (java.lang.Math.abs(error) < EPS) {
            return;
        }
        float v2 = v - 0.5f * error;
        float w2 = w + 0.5f * (error / ratio);
        setLinearComponent(rack.body(), rackAxis, v2);
        setAngularComponent(pinion.body(), pinionAxis, w2);
    }

    private void solvePulley(ConstraintDesc desc, float dt) {
        Ode4jBodyHandle a = bodyRegistry.getHandle(desc.bodyA());
        Ode4jBodyHandle b = bodyRegistry.getHandle(desc.bodyB());
        if (!dynamic(a) || !dynamic(b)) {
            return;
        }
        float ratio = ratioOrDefault(desc.limits().linearLowerLimit(), 1f);
        float ropeLength = java.lang.Math.max(0f, desc.limits().linearUpperLimit());

        Vector3f axisA = normalized(desc.axisA(), new Vector3f(0f, 1f, 0f));
        Vector3f axisB = normalized(desc.axisB(), new Vector3f(0f, 1f, 0f));
        Vector3f posA = toVec(a.body().getPosition());
        Vector3f posB = toVec(b.body().getPosition());
        Vector3f anchorA = desc.pivotA();
        Vector3f anchorB = desc.pivotB();

        float dA = dot(posA.sub(anchorA, new Vector3f()), axisA);
        float dB = dot(posB.sub(anchorB, new Vector3f()), axisB);
        float posError = (dA + ratio * dB) - ropeLength;

        float vA = dot(toVec(a.body().getLinearVel()), axisA);
        float vB = dot(toVec(b.body().getLinearVel()), axisB);
        float velError = (vA + ratio * vB) + (BETA * posError / dt);
        if (java.lang.Math.abs(velError) < EPS) {
            return;
        }
        float vA2 = vA - 0.5f * velError;
        float vB2 = vB - 0.5f * (velError / ratio);
        setLinearComponent(a.body(), axisA, vA2);
        setLinearComponent(b.body(), axisB, vB2);
    }

    private static void setLinearComponent(DBody body, Vector3f axis, float target) {
        Vector3f v = toVec(body.getLinearVel());
        float current = dot(v, axis);
        Vector3f updated = v.add(axis.mul(target - current, new Vector3f()), new Vector3f());
        body.setLinearVel(updated.x(), updated.y(), updated.z());
    }

    private static void setAngularComponent(DBody body, Vector3f axis, float target) {
        Vector3f v = toVec(body.getAngularVel());
        float current = dot(v, axis);
        Vector3f updated = v.add(axis.mul(target - current, new Vector3f()), new Vector3f());
        body.setAngularVel(updated.x(), updated.y(), updated.z());
    }

    private static boolean dynamic(Ode4jBodyHandle h) {
        return h != null && h.isAlive() && h.mode() == BodyMode.DYNAMIC && h.body() != null;
    }

    private static float ratioOrDefault(float value, float fallback) {
        if (!Float.isFinite(value) || java.lang.Math.abs(value) < EPS) {
            return fallback;
        }
        return value;
    }

    private static Vector3f normalized(Vector3f axis, Vector3f fallback) {
        if (axis == null) {
            return fallback;
        }
        float len = axis.length();
        if (!(len > EPS)) {
            return fallback;
        }
        return axis.div(len, new Vector3f());
    }

    private static float dot(Vector3f a, Vector3f b) {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    private static Vector3f toVec(DVector3C v) {
        return new Vector3f((float) v.get0(), (float) v.get1(), (float) v.get2());
    }
}
