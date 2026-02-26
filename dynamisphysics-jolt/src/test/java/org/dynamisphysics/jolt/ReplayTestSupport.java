package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.body.StableRigidBodyId;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsTuning;
import org.dynamisphysics.api.config.PhysicsTuningProfile;
import org.dynamisphysics.api.config.PhysicsTuningResolver;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.dynamisphysics.test.replay.PhysicsReplayRecorder;
import org.dynamisphysics.test.replay.PhysicsReplayRunner;
import org.dynamisphysics.test.replay.ReplayCheckpoint;
import org.dynamisphysics.test.replay.ReplayHash;
import org.dynamisphysics.test.replay.ReplayInputFrame;
import org.dynamisphysics.test.replay.ReplayOp;
import org.dynamisphysics.test.replay.ReproPacket;
import org.dynamisphysics.test.replay.ReproPacketJson;
import org.dynamisphysics.test.replay.ReplayValidationMode;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

final class ReplayTestSupport {
    private ReplayTestSupport() {
    }

    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    static PhysicsWorldConfig deterministicConfig(PhysicsBackend backend) {
        PhysicsWorldConfig d = PhysicsWorldConfig.defaults(backend);
        PhysicsTuning tuning = new PhysicsTuning(
            PhysicsTuningProfile.DETERMINISTIC,
            true,
            1,
            null,
            null,
            null
        );
        return new PhysicsWorldConfig(
            backend,
            d.gravity(),
            d.fixedTimeStep(),
            1,
            d.solverIterations(),
            d.maxBodies(),
            d.maxConstraints(),
            d.broadphase(),
            true,
            tuning
        );
    }

    static PhysicsWorldConfig perfConfig(PhysicsBackend backend) {
        PhysicsWorldConfig d = PhysicsWorldConfig.defaults(backend);
        PhysicsTuning tuning = new PhysicsTuning(
            PhysicsTuningProfile.PERF,
            false,
            backend == PhysicsBackend.JOLT ? 8 : 1,
            null,
            null,
            null
        );
        return new PhysicsWorldConfig(
            backend,
            d.gravity(),
            d.fixedTimeStep(),
            1,
            d.solverIterations(),
            d.maxBodies(),
            d.maxConstraints(),
            d.broadphase(),
            false,
            tuning
        );
    }

    static List<RigidBodyHandle> setupSimpleScene(PhysicsWorld world) {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(100f, 0.1f, 100f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .build());
        List<RigidBodyHandle> bodies = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            bodies.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.25f), 1f)
                .worldTransform(new Matrix4f().translation((i % 5) - 2f, 2.0f + (i / 5), 0f))
                .build()));
        }
        world.setGravity(new Vector3f(0f, -9.81f, 0f));
        return bodies;
    }

    static int bodyId(RigidBodyHandle handle) {
        if (handle instanceof StableRigidBodyId stable) {
            return stable.bodyId();
        }
        throw new IllegalStateException("Replay test requires StableRigidBodyId handle");
    }

    static PhysicsReplayRecorder newRecorder(PhysicsWorld world, PhysicsBackend backend, PhysicsWorldConfig config) {
        return new PhysicsReplayRecorder(
            world,
            System.getProperty("project.version", "0.3.1-SNAPSHOT"),
            backend,
            PhysicsTuningResolver.resolve(config),
            config.fixedTimeStep(),
            config.maxSubSteps(),
            "ReplayRoundTrip",
            Map.of("bodies", 11),
            42L
        );
    }

    static PhysicsReplayRunner.ReplayResult runPacket(PhysicsBackend backend, PhysicsWorldConfig config, ReproPacket packet) {
        PhysicsWorld replayWorld = PhysicsWorldFactory.create(config);
        try {
            return PhysicsReplayRunner.run(replayWorld, packet, ReplayResolvers.forWorld(backend, replayWorld));
        } finally {
            replayWorld.destroy();
        }
    }

    static ReproPacket roundTripJson(ReproPacket packet) {
        return ReproPacketJson.fromJson(ReproPacketJson.toJson(packet));
    }

    static ReproPacket withValidationMode(ReproPacket packet, ReplayValidationMode mode) {
        return new ReproPacket(
            packet.magic(),
            packet.formatVersion(),
            packet.createdUtc(),
            packet.engineVersion(),
            packet.backend(),
            packet.tuning(),
            packet.worldConfig(),
            packet.scene(),
            mode,
            packet.invariants(),
            packet.seed(),
            packet.initialSnapshotB64(),
            packet.inputs(),
            packet.checkpoints()
        );
    }

    static ReproPacket canonicalizeCheckpointsFromFreshRestore(
        PhysicsBackend backend,
        PhysicsWorldConfig config,
        ReproPacket packet
    ) {
        // Canonical STRICT rule:
        // reference checkpoints must come from the same history class as replay:
        // fresh world -> restore(packet.initialSnapshot) -> apply inputs -> step -> hash.
        // Do not derive STRICT checkpoints from a same-world post-build history.
        if (packet.checkpoints().isEmpty()) {
            return packet;
        }
        TreeSet<Integer> checkpointSteps = new TreeSet<>();
        for (ReplayCheckpoint checkpoint : packet.checkpoints()) {
            checkpointSteps.add(checkpoint.step());
        }
        int maxStep = checkpointSteps.last();
        Map<Integer, List<ReplayOp>> opsByStep = new HashMap<>();
        for (ReplayInputFrame frame : packet.inputs()) {
            opsByStep.put(frame.step(), frame.ops());
        }
        List<ReplayCheckpoint> canonical = new ArrayList<>();
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            world.restore(java.util.Base64.getDecoder().decode(packet.initialSnapshotB64()));
            var resolver = ReplayResolvers.forWorld(backend, world);
            for (int step = 0; step < maxStep; step++) {
                List<ReplayOp> ops = opsByStep.get(step);
                if (ops != null) {
                    for (ReplayOp op : ops) {
                        applyOp(world, resolver, op);
                    }
                }
                world.step(packet.worldConfig().fixedTimeStep());
                int completed = step + 1;
                if (checkpointSteps.contains(completed)) {
                    canonical.add(new ReplayCheckpoint(completed, ReplayHash.sha256Hex(world.snapshot())));
                }
            }
        } finally {
            world.destroy();
        }
        canonical.sort(Comparator.comparingInt(ReplayCheckpoint::step));
        return new ReproPacket(
            packet.magic(),
            packet.formatVersion(),
            packet.createdUtc(),
            packet.engineVersion(),
            packet.backend(),
            packet.tuning(),
            packet.worldConfig(),
            packet.scene(),
            packet.validationMode(),
            packet.invariants(),
            packet.seed(),
            packet.initialSnapshotB64(),
            packet.inputs(),
            canonical
        );
    }

    private static void applyOp(PhysicsWorld world, PhysicsReplayRunner.ReplayHandleResolver resolver, ReplayOp op) {
        if (op instanceof ReplayOp.ApplyImpulseOp impulse) {
            world.applyImpulse(resolver.rigidBody(impulse.rigidBodyId()), impulse.impulse().toVector3f(), impulse.worldPoint().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.ApplyForceOp force) {
            world.applyForce(resolver.rigidBody(force.rigidBodyId()), force.force().toVector3f(), force.worldPoint().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.ApplyTorqueOp torque) {
            world.applyTorque(resolver.rigidBody(torque.rigidBodyId()), torque.torque().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.SetVelocityOp vel) {
            world.setVelocity(resolver.rigidBody(vel.rigidBodyId()), vel.linear().toVector3f(), vel.angular().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.TeleportOp teleport) {
            world.teleport(resolver.rigidBody(teleport.rigidBodyId()), teleport.position().toVector3f(), teleport.orientation().toQuaternionf());
            return;
        }
        if (op instanceof ReplayOp.ApplyThrottleOp throttle) {
            world.applyThrottle(resolver.vehicle(throttle.vehicleId()), throttle.throttle());
            return;
        }
        if (op instanceof ReplayOp.ApplyBrakeOp brake) {
            world.applyBrake(resolver.vehicle(brake.vehicleId()), brake.brake());
            return;
        }
        if (op instanceof ReplayOp.ApplySteeringOp steer) {
            world.applySteering(resolver.vehicle(steer.vehicleId()), steer.steeringAngle());
            return;
        }
        if (op instanceof ReplayOp.ApplyHandbrakeOp hb) {
            world.applyHandbrake(resolver.vehicle(hb.vehicleId()), hb.engaged());
            return;
        }
        if (op instanceof ReplayOp.MoveCharacterOp move) {
            world.moveCharacter(resolver.character(move.characterId()), move.velocity().toVector3f());
            return;
        }
        if (op instanceof ReplayOp.JumpCharacterOp jump) {
            world.jumpCharacter(resolver.character(jump.characterId()), jump.impulse());
            return;
        }
        if (op instanceof ReplayOp.ActivateRagdollOp on) {
            world.activateRagdoll(resolver.ragdoll(on.ragdollId()), on.blendInSeconds());
            return;
        }
        if (op instanceof ReplayOp.DeactivateRagdollOp off) {
            world.deactivateRagdoll(resolver.ragdoll(off.ragdollId()));
            return;
        }
        if (op instanceof ReplayOp.SetRagdollBlendTargetOp) {
            throw new UnsupportedOperationException("Cannot canonicalize SetRagdollBlendTargetOp without pose reconstruction");
        }
        throw new IllegalArgumentException("Unknown replay op: " + op.getClass().getName());
    }
}
