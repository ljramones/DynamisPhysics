package org.dynamisphysics.test.replay;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.List;

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "type")
@JsonSubTypes({
    @JsonSubTypes.Type(value = ReplayOp.ApplyImpulseOp.class, name = "applyImpulse"),
    @JsonSubTypes.Type(value = ReplayOp.ApplyForceOp.class, name = "applyForce"),
    @JsonSubTypes.Type(value = ReplayOp.ApplyTorqueOp.class, name = "applyTorque"),
    @JsonSubTypes.Type(value = ReplayOp.SetVelocityOp.class, name = "setVelocity"),
    @JsonSubTypes.Type(value = ReplayOp.TeleportOp.class, name = "teleport"),
    @JsonSubTypes.Type(value = ReplayOp.ApplyThrottleOp.class, name = "applyThrottle"),
    @JsonSubTypes.Type(value = ReplayOp.ApplyBrakeOp.class, name = "applyBrake"),
    @JsonSubTypes.Type(value = ReplayOp.ApplySteeringOp.class, name = "applySteering"),
    @JsonSubTypes.Type(value = ReplayOp.ApplyHandbrakeOp.class, name = "applyHandbrake"),
    @JsonSubTypes.Type(value = ReplayOp.MoveCharacterOp.class, name = "moveCharacter"),
    @JsonSubTypes.Type(value = ReplayOp.JumpCharacterOp.class, name = "jumpCharacter"),
    @JsonSubTypes.Type(value = ReplayOp.ActivateRagdollOp.class, name = "activateRagdoll"),
    @JsonSubTypes.Type(value = ReplayOp.DeactivateRagdollOp.class, name = "deactivateRagdoll"),
    @JsonSubTypes.Type(value = ReplayOp.SetRagdollBlendTargetOp.class, name = "setRagdollBlendTarget")
})
public sealed interface ReplayOp permits
    ReplayOp.ApplyImpulseOp,
    ReplayOp.ApplyForceOp,
    ReplayOp.ApplyTorqueOp,
    ReplayOp.SetVelocityOp,
    ReplayOp.TeleportOp,
    ReplayOp.ApplyThrottleOp,
    ReplayOp.ApplyBrakeOp,
    ReplayOp.ApplySteeringOp,
    ReplayOp.ApplyHandbrakeOp,
    ReplayOp.MoveCharacterOp,
    ReplayOp.JumpCharacterOp,
    ReplayOp.ActivateRagdollOp,
    ReplayOp.DeactivateRagdollOp,
    ReplayOp.SetRagdollBlendTargetOp {

    record ApplyImpulseOp(int rigidBodyId, Vec3 impulse, Vec3 worldPoint) implements ReplayOp {}

    record ApplyForceOp(int rigidBodyId, Vec3 force, Vec3 worldPoint) implements ReplayOp {}

    record ApplyTorqueOp(int rigidBodyId, Vec3 torque) implements ReplayOp {}

    record SetVelocityOp(int rigidBodyId, Vec3 linear, Vec3 angular) implements ReplayOp {}

    record TeleportOp(int rigidBodyId, Vec3 position, Quat orientation) implements ReplayOp {}

    record ApplyThrottleOp(int vehicleId, float throttle) implements ReplayOp {}

    record ApplyBrakeOp(int vehicleId, float brake) implements ReplayOp {}

    record ApplySteeringOp(int vehicleId, float steeringAngle) implements ReplayOp {}

    record ApplyHandbrakeOp(int vehicleId, boolean engaged) implements ReplayOp {}

    record MoveCharacterOp(int characterId, Vec3 velocity) implements ReplayOp {}

    record JumpCharacterOp(int characterId, float impulse) implements ReplayOp {}

    record ActivateRagdollOp(int ragdollId, float blendInSeconds) implements ReplayOp {}

    record DeactivateRagdollOp(int ragdollId) implements ReplayOp {}

    record SetRagdollBlendTargetOp(int ragdollId, float alpha, String poseHint) implements ReplayOp {}

    record Vec3(float x, float y, float z) {
        public static Vec3 of(Vector3f v) {
            return new Vec3(v.x, v.y, v.z);
        }

        public Vector3f toVector3f() {
            return new Vector3f(x, y, z);
        }
    }

    record Quat(float x, float y, float z, float w) {
        public static Quat of(Quaternionf q) {
            return new Quat(q.x, q.y, q.z, q.w);
        }

        public Quaternionf toQuaternionf() {
            return new Quaternionf(x, y, z, w);
        }
    }

    static ReplayInputFrame frame(int step, List<ReplayOp> ops) {
        return new ReplayInputFrame(step, ops);
    }
}
