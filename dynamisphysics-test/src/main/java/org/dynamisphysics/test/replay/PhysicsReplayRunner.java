package org.dynamisphysics.test.replay;

import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Base64;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class PhysicsReplayRunner {
    private PhysicsReplayRunner() {}

    public static ReplayResult run(PhysicsWorld world, ReproPacket packet) {
        return run(world, packet, null);
    }

    public static ReplayResult run(PhysicsWorld world, ReproPacket packet, ReplayHandleResolver resolver) {
        try {
            ReplayPacketSchema.validate(packet);
        } catch (IllegalArgumentException e) {
            return ReplayResult.failed(0, e.getMessage());
        }

        byte[] snapshot = Base64.getDecoder().decode(packet.initialSnapshotB64());
        ReplayValidationMode mode = packet.validationMode() != null
            ? packet.validationMode()
            : (packet.tuning().deterministic() ? ReplayValidationMode.STRICT : ReplayValidationMode.BEHAVIOURAL);
        if (Boolean.getBoolean("physics.replay.debug.provenance")) {
            printProvenance(packet, mode);
        }
        world.restore(snapshot);
        ReplayInvariants invariants = packet.invariants() != null ? packet.invariants() : ReplayInvariants.defaults();
        int initialBodyCount = world.getStats().bodyCount();

        Map<Integer, List<ReplayOp>> opsByStep = new HashMap<>();
        Set<Integer> touchedBodyIds = new HashSet<>();
        for (ReplayInputFrame frame : packet.inputs()) {
            opsByStep.put(frame.step(), frame.ops());
            collectBodyIds(frame.ops(), touchedBodyIds);
        }

        Map<Integer, String> checkpoints = new HashMap<>();
        int maxStep = 0;
        for (ReplayCheckpoint checkpoint : packet.checkpoints()) {
            checkpoints.put(checkpoint.step(), checkpoint.sha256());
            maxStep = Math.max(maxStep, checkpoint.step());
        }
        for (ReplayInputFrame frame : packet.inputs()) {
            maxStep = Math.max(maxStep, frame.step() + 1);
        }

        for (int step = 0; step < maxStep; step++) {
            List<ReplayOp> ops = opsByStep.get(step);
            if (ops != null && !ops.isEmpty()) {
                if (resolver == null) {
                    return ReplayResult.failed(step, "Replay input ops present but no resolver was provided");
                }
                for (ReplayOp op : ops) {
                    applyOp(world, resolver, op);
                }
            }
            world.step(packet.worldConfig().fixedTimeStep());
            int completedStep = step + 1;
            String expected = checkpoints.get(completedStep);
            if (expected != null) {
                String actual = ReplayHash.sha256Hex(world.snapshot());
                if (!expected.equals(actual) && mode == ReplayValidationMode.STRICT) {
                    return ReplayResult.failed(
                        completedStep,
                        "Checkpoint hash mismatch at step " + completedStep + " expected=" + expected + " actual=" + actual
                    );
                }
                if (mode == ReplayValidationMode.BEHAVIOURAL) {
                    String invariantError = checkInvariants(world, resolver, invariants, touchedBodyIds, initialBodyCount);
                    if (invariantError != null) {
                        return ReplayResult.failed(completedStep, invariantError);
                    }
                }
            }
        }
        if (mode == ReplayValidationMode.BEHAVIOURAL) {
            String invariantError = checkInvariants(world, resolver, invariants, touchedBodyIds, initialBodyCount);
            if (invariantError != null) {
                return ReplayResult.failed(maxStep, invariantError);
            }
        }
        return ReplayResult.ok();
    }

    private static String checkInvariants(
        PhysicsWorld world,
        ReplayHandleResolver resolver,
        ReplayInvariants invariants,
        Set<Integer> touchedBodyIds,
        int initialBodyCount
    ) {
        if (invariants.requireBodyCountStable() && world.getStats().bodyCount() != initialBodyCount) {
            return "Behavioural invariant failed: body count changed initial=" + initialBodyCount
                + " now=" + world.getStats().bodyCount();
        }
        if (resolver == null) {
            return null;
        }
        for (int bodyId : touchedBodyIds) {
            BodyState state = world.getBodyState(resolver.rigidBody(bodyId));
            if (invariants.requireFinite()) {
                if (!finite(state.position()) || !finite(state.linearVelocity()) || !finite(state.angularVelocity())) {
                    return "Behavioural invariant failed: non-finite body state for bodyId=" + bodyId;
                }
            }
            if (state.position().y < invariants.minY()) {
                return "Behavioural invariant failed: bodyId=" + bodyId + " below minY="
                    + invariants.minY() + " actualY=" + state.position().y;
            }
            float linearSpeed = state.linearVelocity().length();
            float angularSpeed = state.angularVelocity().length();
            if (linearSpeed > invariants.maxSpeed() || angularSpeed > invariants.maxSpeed()) {
                return "Behavioural invariant failed: bodyId=" + bodyId + " exceeds maxSpeed="
                    + invariants.maxSpeed() + " linear=" + linearSpeed + " angular=" + angularSpeed;
            }
        }
        return null;
    }

    private static boolean finite(Vector3f v) {
        return Float.isFinite(v.x) && Float.isFinite(v.y) && Float.isFinite(v.z);
    }

    private static void collectBodyIds(List<ReplayOp> ops, Set<Integer> out) {
        for (ReplayOp op : ops) {
            if (op instanceof ReplayOp.ApplyImpulseOp o) out.add(o.rigidBodyId());
            else if (op instanceof ReplayOp.ApplyForceOp o) out.add(o.rigidBodyId());
            else if (op instanceof ReplayOp.ApplyTorqueOp o) out.add(o.rigidBodyId());
            else if (op instanceof ReplayOp.SetVelocityOp o) out.add(o.rigidBodyId());
            else if (op instanceof ReplayOp.TeleportOp o) out.add(o.rigidBodyId());
        }
    }

    private static void applyOp(PhysicsWorld world, ReplayHandleResolver resolver, ReplayOp op) {
        if (op instanceof ReplayOp.ApplyImpulseOp o) {
            world.applyImpulse(resolver.rigidBody(o.rigidBodyId()), o.impulse().toVector3f(), o.worldPoint().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.ApplyForceOp o) {
            world.applyForce(resolver.rigidBody(o.rigidBodyId()), o.force().toVector3f(), o.worldPoint().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.ApplyTorqueOp o) {
            world.applyTorque(resolver.rigidBody(o.rigidBodyId()), o.torque().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.SetVelocityOp o) {
            world.setVelocity(resolver.rigidBody(o.rigidBodyId()), o.linear().toVector3f(), o.angular().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.TeleportOp o) {
            world.teleport(resolver.rigidBody(o.rigidBodyId()), o.position().toVector3f(), o.orientation().toQuaternionf());
            return;
        }
        if (op instanceof ReplayOp.ApplyThrottleOp o) {
            world.applyThrottle(resolver.vehicle(o.vehicleId()), o.throttle());
            return;
        }
        if (op instanceof ReplayOp.ApplyBrakeOp o) {
            world.applyBrake(resolver.vehicle(o.vehicleId()), o.brake());
            return;
        }
        if (op instanceof ReplayOp.ApplySteeringOp o) {
            world.applySteering(resolver.vehicle(o.vehicleId()), o.steeringAngle());
            return;
        }
        if (op instanceof ReplayOp.ApplyHandbrakeOp o) {
            world.applyHandbrake(resolver.vehicle(o.vehicleId()), o.engaged());
            return;
        }
        if (op instanceof ReplayOp.MoveCharacterOp o) {
            world.moveCharacter(resolver.character(o.characterId()), o.velocity().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.JumpCharacterOp o) {
            world.jumpCharacter(resolver.character(o.characterId()), o.impulse());
            return;
        }
        if (op instanceof ReplayOp.ActivateRagdollOp o) {
            world.activateRagdoll(resolver.ragdoll(o.ragdollId()), o.blendInSeconds());
            return;
        }
        if (op instanceof ReplayOp.DeactivateRagdollOp o) {
            world.deactivateRagdoll(resolver.ragdoll(o.ragdollId()));
            return;
        }
        if (op instanceof ReplayOp.SetRagdollBlendTargetOp) {
            throw new UnsupportedOperationException("SetRagdollBlendTargetOp replay requires AnimisPose reconstruction");
        }
        throw new IllegalArgumentException("Unknown replay op: " + op.getClass().getName());
    }

    private static void printProvenance(ReproPacket packet, ReplayValidationMode mode) {
        String gitSha = System.getProperty("git.commit.id.abbrev", System.getProperty("physics.gitSha", "unknown"));
        System.out.println("[ReplayProvenance] schema=" + packet.formatVersion()
            + " magic=" + packet.magic()
            + " fingerprint=" + ReproPacket.SCHEMA_FINGERPRINT
            + " backend=" + packet.backend()
            + " validationMode=" + mode
            + " deterministic=" + packet.tuning().deterministic()
            + " threads=" + packet.tuning().threads()
            + " allocator=" + packet.tuning().allocatorMode() + "(" + packet.tuning().allocatorMb() + "MB)"
            + " solverIterations=" + packet.tuning().solverIterations()
            + " jdk=" + System.getProperty("java.version", "unknown")
            + " os=" + System.getProperty("os.name", "unknown") + " " + System.getProperty("os.version", "unknown")
            + " gitSha=" + gitSha);
    }

    public interface ReplayHandleResolver {
        RigidBodyHandle rigidBody(int id);

        VehicleHandle vehicle(int id);

        CharacterHandle character(int id);

        RagdollHandle ragdoll(int id);
    }

    public record ReplayResult(boolean success, int failedStep, String message) {
        public static ReplayResult ok() {
            return new ReplayResult(true, -1, "OK");
        }

        public static ReplayResult failed(int failedStep, String message) {
            return new ReplayResult(false, failedStep, message);
        }
    }
}
