package org.dynamisphysics.ode4j.ragdoll;

import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.RagdollJointDesc;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.world.GetUpPoseHint;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.dynamisphysics.ode4j.constraint.Ode4jConstraintRegistry;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;
import org.vectrix.physics.PdControllersf;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toOde;

public final class Ode4jRagdollSystem {
    private static final class Instance {
        private final Ode4jRagdollHandle handle;
        private final RagdollDescriptor descriptor;
        private final List<ConstraintHandle> constraints;

        private static final float REST_LINEAR_SPEED = 0.05f;
        private static final float REST_ANGULAR_SPEED = 0.05f;

        private AnimisPose targetPose;
        private float blendAlpha;
        private boolean hintEmitted;

        private Instance(Ode4jRagdollHandle handle, RagdollDescriptor descriptor, List<ConstraintHandle> constraints) {
            this.handle = handle;
            this.descriptor = descriptor;
            this.constraints = constraints;
            this.targetPose = AnimisPose.EMPTY;
            this.blendAlpha = 0f;
            this.hintEmitted = false;
        }
    }

    private final Ode4jBodyRegistry bodyRegistry;
    private final Ode4jConstraintRegistry constraintRegistry;
    private final Map<RagdollHandle, Instance> ragdolls = new LinkedHashMap<>();
    private int nextRagdollId = 1;

    public Ode4jRagdollSystem(Ode4jBodyRegistry bodyRegistry, Ode4jConstraintRegistry constraintRegistry) {
        this.bodyRegistry = bodyRegistry;
        this.constraintRegistry = constraintRegistry;
    }

    public RagdollHandle spawn(RagdollDescriptor descriptor, AnimisPose initialPose) {
        String ragdollId = "ragdoll-" + nextRagdollId++;
        Map<String, RigidBodyHandle> bones = new LinkedHashMap<>();
        for (RagdollBoneDesc bone : descriptor.bones()) {
            Matrix4f boneTransform = initialPose.boneWorldTransform(bone.animisBoneName());
            RigidBodyHandle body = bodyRegistry.spawn(RigidBodyConfig.builder(bone.shape(), bone.mass())
                .worldTransform(boneTransform)
                .build());
            bones.put(bone.animisBoneName(), body);
        }

        Ode4jRagdollHandle handle = new Ode4jRagdollHandle(
            ragdollId,
            bones,
            boneName -> {
                RigidBodyHandle b = bones.get(boneName);
                return b == null ? BodyState.ZERO : bodyRegistry.getState(b);
            }
        );

        List<ConstraintHandle> constraints = new ArrayList<>();
        for (RagdollJointDesc joint : descriptor.joints()) {
            RigidBodyHandle parent = bones.get(joint.parentBone());
            RigidBodyHandle child = bones.get(joint.childBone());
            if (parent == null || child == null) {
                continue;
            }
            ConstraintType type = joint.jointType() != null ? joint.jointType() : ConstraintType.BALL_SOCKET;
            constraints.add(constraintRegistry.add(new ConstraintDesc(
                type,
                parent,
                child,
                new Vector3f(),
                new Vector3f(),
                new Vector3f(0f, 1f, 0f),
                new Vector3f(0f, 1f, 0f),
                joint.limits(),
                ConstraintMotor.off(),
                0f,
                0f
            )));
        }

        Instance instance = new Instance(handle, descriptor, constraints);
        instance.targetPose = initialPose;
        ragdolls.put(handle, instance);
        return handle;
    }

    public void destroy(RagdollHandle handle) {
        Instance instance = ragdolls.remove(handle);
        if (instance == null) {
            return;
        }
        for (ConstraintHandle c : instance.constraints) {
            constraintRegistry.remove(c);
        }
        for (RigidBodyHandle body : instance.handle.bones().values()) {
            bodyRegistry.destroy(body);
        }
        instance.handle.kill();
    }

    public void activate(RagdollHandle handle, float blendInSeconds) {
        Instance i = lookup(handle);
        i.blendAlpha = 1f;
    }

    public void deactivate(RagdollHandle handle) {
        Instance i = lookup(handle);
        i.blendAlpha = 0f;
    }

    public void setBlendTarget(RagdollHandle handle, AnimisPose pose, float alpha) {
        Instance i = lookup(handle);
        i.targetPose = pose != null ? pose : AnimisPose.EMPTY;
        i.blendAlpha = org.vectrix.core.Math.clamp(alpha, 0f, 1f);
        i.hintEmitted = false;
    }

    public void stepAll(float dt) {
        for (Instance instance : ragdolls.values()) {
            step(instance, dt);
        }
    }

    private void step(Instance instance, float dt) {
        if (!instance.handle.isAlive()) {
            return;
        }

        float alpha = instance.blendAlpha;
        AnimisPose targetPose = instance.targetPose;

        for (RagdollBoneDesc bone : instance.descriptor.bones()) {
            RigidBodyHandle h = instance.handle.bones().get(bone.animisBoneName());
            if (h == null) {
                continue;
            }
            BodyState current = bodyRegistry.getState(h);

            Matrix4f targetMatrix = targetPose.boneWorldTransform(bone.animisBoneName());
            Vector3f targetPos = new Vector3f();
            targetMatrix.getTranslation(targetPos);
            Quaternionf targetOri = new Quaternionf();
            targetMatrix.getUnnormalizedRotation(targetOri);

            if (alpha <= 0.001f) {
                bodyRegistry.setState(h, new BodyState(targetPos, targetOri, new Vector3f(), new Vector3f(), false));
                continue;
            }

            Quaternionf blendedTarget = alpha < 0.999f
                ? new Quaternionf(current.orientation()).slerp(targetOri, alpha, new Quaternionf())
                : targetOri;

            Vector3f angularError = Quaternionf.angularError(current.orientation(), blendedTarget, new Vector3f());
            Vector3f torque = PdControllersf.torque(
                angularError,
                bone.kp(),
                current.angularVelocity(),
                bone.kd(),
                new Vector3f()
            );
            torque.mul(alpha);
            Vector3f clamped = PdControllersf.clampTorque(torque, bone.maxTorque(), new Vector3f());

            Ode4jBodyHandle oh = bodyRegistry.getHandle(h);
            if (oh != null && oh.body() != null) {
                oh.body().addTorque(toOde(clamped));
            }
        }

        maybeEmitGetUpHint(instance);
    }

    private void maybeEmitGetUpHint(Instance instance) {
        if (instance.descriptor.getUpHintListener() == null || instance.hintEmitted || instance.blendAlpha < 0.95f) {
            return;
        }

        boolean allRest = true;
        for (RigidBodyHandle body : instance.handle.bones().values()) {
            BodyState s = bodyRegistry.getState(body);
            if (s.linearVelocity().length() > Instance.REST_LINEAR_SPEED
                || s.angularVelocity().length() > Instance.REST_ANGULAR_SPEED) {
                allRest = false;
                break;
            }
        }
        if (!allRest) {
            return;
        }

        String spineName = instance.handle.bones().containsKey("spine")
            ? "spine"
            : instance.handle.bones().keySet().iterator().next();
        BodyState spineState = instance.handle.getBoneState(spineName);
        Vector3f spineUp = spineState.orientation().transform(new Vector3f(0f, 1f, 0f), new Vector3f());
        boolean faceDown = spineUp.y() < 0f;

        instance.descriptor.getUpHintListener().accept(new GetUpPoseHint(
            instance.handle.id(),
            faceDown,
            new Quaternionf(spineState.orientation())
        ));
        instance.hintEmitted = true;
    }

    private Instance lookup(RagdollHandle handle) {
        Instance instance = ragdolls.get(handle);
        if (instance == null) {
            throw new IllegalArgumentException("Unknown ragdoll handle: " + handle);
        }
        return instance;
    }
}
