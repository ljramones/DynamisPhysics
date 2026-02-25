package org.dynamisphysics.jolt.constraint;

import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.Constraint;
import com.github.stephengold.joltjni.MotorSettings;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.SixDofConstraint;
import com.github.stephengold.joltjni.SixDofConstraintSettings;
import com.github.stephengold.joltjni.SpringSettings;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EAxis;
import com.github.stephengold.joltjni.enumerate.EConstraintSpace;
import com.github.stephengold.joltjni.enumerate.EMotorState;
import com.github.stephengold.joltjni.enumerate.ESpringMode;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.vectrix.core.Vector3f;

final class JoltConstraintFactory {
    private static final float DEFAULT_STIFFNESS = 40f;

    private JoltConstraintFactory() {
    }

    static JoltConstraintHandle create(
        int constraintId,
        ConstraintDesc desc,
        PhysicsSystem physicsSystem,
        JoltBodyHandle bodyA,
        JoltBodyHandle bodyB
    ) {
        Constraint constraint = switch (desc.type()) {
            case SIX_DOF_SPRING -> createSixDofSpring(desc, physicsSystem, bodyA, bodyB);
            default -> throw new UnsupportedOperationException("Jolt constraint type not yet supported: " + desc.type());
        };
        return new JoltConstraintHandle(constraintId, desc, constraint);
    }

    private static Constraint createSixDofSpring(
        ConstraintDesc desc,
        PhysicsSystem physicsSystem,
        JoltBodyHandle bodyA,
        JoltBodyHandle bodyB
    ) {
        SixDofConstraintSettings settings = new SixDofConstraintSettings();
        settings.setSpace(EConstraintSpace.WorldSpace);
        settings.makeFixedAxis(EAxis.TranslationX);
        settings.makeFreeAxis(EAxis.TranslationY);
        settings.makeFixedAxis(EAxis.TranslationZ);
        settings.makeFixedAxis(EAxis.RotationX);
        settings.makeFixedAxis(EAxis.RotationY);
        settings.makeFixedAxis(EAxis.RotationZ);
        settings.setPosition1(toRVec3(desc.pivotA()));
        settings.setPosition2(toRVec3(desc.pivotB()));
        settings.setAxisX1(new Vec3(1f, 0f, 0f));
        settings.setAxisX2(new Vec3(1f, 0f, 0f));
        settings.setAxisY1(new Vec3(0f, 1f, 0f));
        settings.setAxisY2(new Vec3(0f, 1f, 0f));

        BodyLockRead lockA = new BodyLockRead(physicsSystem.getBodyLockInterface(), bodyA.joltBodyId());
        BodyLockRead lockB = new BodyLockRead(physicsSystem.getBodyLockInterface(), bodyB.joltBodyId());
        try {
            if (!lockA.succeeded() || !lockB.succeeded()) {
                throw new IllegalStateException("Failed to lock bodies for SIX_DOF_SPRING creation");
            }
            Body a = (Body) lockA.getBody();
            Body b = (Body) lockB.getBody();
            SixDofConstraint spring = (SixDofConstraint) settings.create(a, b);

            float k = finitePositive(desc.motor().maxForce(), DEFAULT_STIFFNESS);
            float c = finitePositive(desc.motor().maxTorque(), 2f * (float) java.lang.Math.sqrt(k));
            SpringSettings s = new SpringSettings()
                .setMode(ESpringMode.StiffnessAndDamping)
                .setStiffness(k)
                .setDamping(c);
            spring.setLimitsSpringSettings(EAxis.TranslationY, s);

            MotorSettings motorSettings = spring.getMotorSettings(EAxis.TranslationY);
            motorSettings.getSpringSettings()
                .setMode(ESpringMode.StiffnessAndDamping)
                .setStiffness(k)
                .setDamping(c);
            motorSettings.setForceLimit(k * 8f);
            spring.setMotorState(EAxis.TranslationY, EMotorState.Position);
            spring.setTargetPositionCs(new Vec3(0f, desc.motor().targetPosition(), 0f));
            spring.setTargetVelocityCs(new Vec3(0f, desc.motor().targetVelocity(), 0f));
            return spring;
        } finally {
            lockB.releaseLock();
            lockA.releaseLock();
        }
    }

    static void setMotorTarget(JoltConstraintHandle handle, float target) {
        if (handle.type() != ConstraintType.SIX_DOF_SPRING) {
            return;
        }
        if (handle.constraint() instanceof SixDofConstraint c) {
            c.setTargetPositionCs(new Vec3(0f, target, 0f));
        }
    }

    private static RVec3 toRVec3(Vector3f v) {
        return new RVec3(v.x(), v.y(), v.z());
    }

    private static float finitePositive(float value, float fallback) {
        if (Float.isFinite(value) && value > 0f) {
            return value;
        }
        return fallback;
    }
}
