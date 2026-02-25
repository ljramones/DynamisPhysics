package org.dynamisphysics.jolt.ragdoll;

import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.Constraint;
import com.github.stephengold.joltjni.HingeConstraintSettings;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.PointConstraintSettings;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.TwoBodyConstraintSettings;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EConstraintSpace;
import org.dynamisphysics.api.RagdollJointDesc;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

final class JoltRagdollBuilder {
    private JoltRagdollBuilder() {
    }

    static List<Constraint> buildConstraints(
        PhysicsSystem physicsSystem,
        Map<String, JoltBodyHandle> bones,
        List<RagdollJointDesc> joints
    ) {
        List<Constraint> constraints = new ArrayList<>();
        for (RagdollJointDesc joint : joints) {
            JoltBodyHandle parent = bones.get(joint.parentBone());
            JoltBodyHandle child = bones.get(joint.childBone());
            if (parent == null || child == null) {
                continue;
            }
            Constraint constraint = createConstraint(physicsSystem, parent, child, joint.jointType());
            if (constraint != null) {
                constraints.add(constraint);
            }
        }
        return constraints;
    }

    private static Constraint createConstraint(
        PhysicsSystem physicsSystem,
        JoltBodyHandle parent,
        JoltBodyHandle child,
        ConstraintType type
    ) {
        BodyLockRead parentLock = new BodyLockRead(physicsSystem.getBodyLockInterface(), parent.joltBodyId());
        BodyLockRead childLock = new BodyLockRead(physicsSystem.getBodyLockInterface(), child.joltBodyId());
        try {
            if (!parentLock.succeeded() || !childLock.succeeded()) {
                return null;
            }
            Body parentBody = (Body) parentLock.getBody();
            Body childBody = (Body) childLock.getBody();

            Vector3f parentPos = new Vector3f((float) parentBody.getPosition().xx(), (float) parentBody.getPosition().yy(), (float) parentBody.getPosition().zz());
            Vector3f childPos = new Vector3f((float) childBody.getPosition().xx(), (float) childBody.getPosition().yy(), (float) childBody.getPosition().zz());
            Vector3f anchor = parentPos.add(childPos, new Vector3f()).mul(0.5f);

            TwoBodyConstraintSettings settings = switch (type) {
                case HINGE -> makeHingeSettings(anchor);
                default -> makePointSettings(anchor);
            };
            return (Constraint) settings.create(parentBody, childBody);
        } finally {
            childLock.releaseLock();
            parentLock.releaseLock();
        }
    }

    private static PointConstraintSettings makePointSettings(Vector3f anchor) {
        PointConstraintSettings settings = new PointConstraintSettings();
        settings.setSpace(EConstraintSpace.WorldSpace);
        RVec3 p = new RVec3(anchor.x(), anchor.y(), anchor.z());
        settings.setPoint1(p);
        settings.setPoint2(p);
        return settings;
    }

    private static HingeConstraintSettings makeHingeSettings(Vector3f anchor) {
        HingeConstraintSettings settings = new HingeConstraintSettings();
        settings.setSpace(EConstraintSpace.WorldSpace);
        RVec3 p = new RVec3(anchor.x(), anchor.y(), anchor.z());
        settings.setPoint1(p);
        settings.setPoint2(p);
        settings.setHingeAxis1(new Vec3(0f, 1f, 0f));
        settings.setHingeAxis2(new Vec3(0f, 1f, 0f));
        settings.setNormalAxis1(new Vec3(1f, 0f, 0f));
        settings.setNormalAxis2(new Vec3(1f, 0f, 0f));
        return settings;
    }
}

