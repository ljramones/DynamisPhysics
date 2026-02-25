package org.dynamisphysics.jolt.ragdoll;

import com.github.stephengold.joltjni.Constraint;
import com.github.stephengold.joltjni.PhysicsSystem;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.world.GetUpPoseHint;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.dynamisphysics.jolt.world.JoltConversions.toVec3;

public final class JoltRagdollSystem {
    static final class Instance {
        private final JoltRagdollHandle handle;
        private final RagdollDescriptor descriptor;
        private final Map<String, JoltBodyHandle> boneBodies;
        private final List<Constraint> constraints;
        private final JoltRagdollRestDetector restDetector;

        private AnimisPose targetPose;
        private float blendAlpha;
        private boolean hintEmitted;

        private Instance(
            JoltRagdollHandle handle,
            RagdollDescriptor descriptor,
            Map<String, JoltBodyHandle> boneBodies,
            List<Constraint> constraints,
            AnimisPose targetPose
        ) {
            this.handle = handle;
            this.descriptor = descriptor;
            this.boneBodies = boneBodies;
            this.constraints = constraints;
            this.targetPose = targetPose;
            this.blendAlpha = 0f;
            this.hintEmitted = false;
            this.restDetector = new JoltRagdollRestDetector();
        }

        JoltRagdollHandle handle() {
            return handle;
        }

        RagdollDescriptor descriptor() {
            return descriptor;
        }
    }

    private final PhysicsSystem physicsSystem;
    private final JoltBodyRegistry bodyRegistry;
    private final Map<RagdollHandle, Instance> ragdolls = new LinkedHashMap<>();
    private int nextRagdollId = 1;

    public JoltRagdollSystem(PhysicsSystem physicsSystem, JoltBodyRegistry bodyRegistry) {
        this.physicsSystem = physicsSystem;
        this.bodyRegistry = bodyRegistry;
    }

    public RagdollHandle spawn(RagdollDescriptor descriptor, AnimisPose initialPose) {
        String ragdollId = "ragdoll-" + nextRagdollId++;
        Map<String, JoltBodyHandle> boneBodies = new LinkedHashMap<>();
        Map<String, RigidBodyHandle> publicBones = new LinkedHashMap<>();

        for (RagdollBoneDesc bone : descriptor.bones()) {
            Matrix4f world = initialPose.boneWorldTransform(bone.animisBoneName());
            JoltBodyHandle body = bodyRegistry.spawn(RigidBodyConfig.builder(bone.shape(), bone.mass())
                .worldTransform(world)
                .build());
            boneBodies.put(bone.animisBoneName(), body);
            publicBones.put(bone.animisBoneName(), body);
        }

        JoltRagdollHandle handle = new JoltRagdollHandle(
            ragdollId,
            publicBones,
            boneName -> {
                RigidBodyHandle body = publicBones.get(boneName);
                return body == null ? BodyState.ZERO : bodyRegistry.getState(body);
            }
        );

        List<Constraint> constraints = JoltRagdollBuilder.buildConstraints(physicsSystem, boneBodies, descriptor.joints());
        for (Constraint constraint : constraints) {
            physicsSystem.addConstraint(constraint);
        }

        ragdolls.put(handle, new Instance(
            handle,
            descriptor,
            boneBodies,
            constraints,
            initialPose != null ? initialPose : AnimisPose.EMPTY
        ));
        return handle;
    }

    public void destroy(RagdollHandle handle) {
        Instance instance = ragdolls.remove(handle);
        if (instance == null) {
            return;
        }
        for (Constraint constraint : instance.constraints) {
            physicsSystem.removeConstraint(constraint);
        }
        for (RigidBodyHandle body : instance.handle.bones().values()) {
            bodyRegistry.destroy(body);
        }
        instance.handle.kill();
    }

    public void activate(RagdollHandle handle, float blendInSeconds) {
        Instance instance = lookup(handle);
        instance.blendAlpha = 1f;
        instance.restDetector.reset();
        instance.hintEmitted = false;
    }

    public void deactivate(RagdollHandle handle) {
        Instance instance = lookup(handle);
        instance.blendAlpha = 0f;
        instance.restDetector.reset();
        instance.hintEmitted = false;
    }

    public void setBlendTarget(RagdollHandle handle, AnimisPose pose, float alpha) {
        Instance instance = lookup(handle);
        instance.targetPose = pose != null ? pose : AnimisPose.EMPTY;
        instance.blendAlpha = org.vectrix.core.Math.clamp(alpha, 0f, 1f);
        instance.restDetector.reset();
        instance.hintEmitted = false;
    }

    public void stepAll(float dt) {
        for (Instance instance : ragdolls.values()) {
            step(instance);
        }
    }

    public void clearAll() {
        var snapshot = List.copyOf(ragdolls.keySet());
        for (RagdollHandle handle : snapshot) {
            destroy(handle);
        }
    }

    private void step(Instance instance) {
        if (!instance.handle.isAlive()) {
            return;
        }

        float alpha = instance.blendAlpha;
        AnimisPose targetPose = instance.targetPose;

        for (RagdollBoneDesc bone : instance.descriptor.bones()) {
            JoltBodyHandle body = instance.boneBodies.get(bone.animisBoneName());
            if (body == null) {
                continue;
            }
            BodyState current = bodyRegistry.getState(body);
            Matrix4f targetMatrix = targetPose.boneWorldTransform(bone.animisBoneName());
            Vector3f targetPos = new Vector3f();
            targetMatrix.getTranslation(targetPos);
            Quaternionf targetOri = new Quaternionf();
            targetMatrix.getUnnormalizedRotation(targetOri);

            if (alpha <= 0.001f) {
                bodyRegistry.setState(body, new BodyState(targetPos, targetOri, new Vector3f(), new Vector3f(), false));
                continue;
            }

            Quaternionf blendedTarget = alpha < 0.999f
                ? new Quaternionf(current.orientation()).slerp(targetOri, alpha, new Quaternionf())
                : targetOri;
            Vector3f angularError = Quaternionf.angularError(current.orientation(), blendedTarget, new Vector3f());
            Vector3f torque = JoltPdController.computeTorque(
                angularError,
                bone.kp(),
                current.angularVelocity(),
                bone.kd(),
                bone.maxTorque(),
                alpha
            );
            bodyRegistry.applyTorque(body, toVec3(torque));
        }

        maybeEmitGetUpHint(instance);
    }

    private void maybeEmitGetUpHint(Instance instance) {
        if (instance.descriptor.getUpHintListener() == null || instance.hintEmitted || instance.blendAlpha < 0.95f) {
            return;
        }
        if (!instance.restDetector.update(instance, bodyRegistry)) {
            return;
        }
        String referenceBone = instance.handle.bones().containsKey("spine")
            ? "spine"
            : instance.handle.bones().keySet().stream().findFirst().orElse(null);
        if (referenceBone == null) {
            return;
        }

        BodyState ref = instance.handle.getBoneState(referenceBone);
        Vector3f up = ref.orientation().transform(new Vector3f(0f, 1f, 0f), new Vector3f());
        boolean faceDown = up.y() < 0f;
        instance.descriptor.getUpHintListener().accept(new GetUpPoseHint(
            instance.handle.id(),
            faceDown,
            new Quaternionf(ref.orientation())
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

