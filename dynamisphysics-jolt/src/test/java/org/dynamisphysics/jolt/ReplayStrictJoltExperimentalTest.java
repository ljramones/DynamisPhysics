package org.dynamisphysics.jolt;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsTuningResolver;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.test.replay.PhysicsReplayRecorder;
import org.dynamisphysics.test.replay.PhysicsReplayRunner;
import org.dynamisphysics.test.replay.ReplayHash;
import org.dynamisphysics.test.replay.ReplayInputFrame;
import org.dynamisphysics.test.replay.ReplayOp;
import org.dynamisphysics.test.replay.ReplayValidationMode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.Base64;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.replay.strictJolt", matches = "true")
class ReplayStrictJoltExperimentalTest {
    @BeforeAll
    static void registerBackends() {
        ReplayTestSupport.registerBackends();
    }

    @Test
    void locateJoltStrictReplayDrift() {
        PhysicsBackend backend = PhysicsBackend.JOLT;
        PhysicsWorldConfig config = ReplayTestSupport.deterministicConfig(backend);
        int steps = Integer.getInteger("physics.replay.steps", 1000);
        int checkpoint = Integer.getInteger("physics.replay.checkpoint", steps);
        int hashLimit = Integer.getInteger("physics.replay.hashLimit", steps);
        boolean noOps = Boolean.getBoolean("physics.replay.noOps");
        boolean debug = Boolean.getBoolean("physics.replay.debug");

        if (debug) {
            System.out.println("[replay-debug] resolved=" + PhysicsTuningResolver.resolve(config));
        }

        RecordedScenario recorded = recordScenario(config, steps, checkpoint, hashLimit, noOps);
        if (debug) {
            System.out.println("[replay-debug] packet.tuning=" + recorded.packet().tuning()
                + " packet.validationMode=" + recorded.packet().validationMode());
        }
        PhysicsReplayRunner.ReplayResult replayResult = ReplayTestSupport.runPacket(backend, config, recorded.packet());

        if (replayResult.success()) {
            System.out.println("First failing checkpoint step: NONE");
            System.out.println("First mismatch step: NONE");
            System.out.println("Manual rerun matches expected: true");
            return;
        }

        int failingCheckpoint = replayResult.failedStep();
        int firstMismatchStep = findFirstMismatchStep(config, recorded.packet(), recorded.referenceHashes(), failingCheckpoint, noOps);
        String expectedAtMismatch = requiredHash(recorded.referenceHashes(), firstMismatchStep);
        String replayAtMismatch = runReplayHashAtStep(config, recorded.packet(), firstMismatchStep, noOps);
        String manualAtMismatch = runScriptedReferenceHash(config, recorded.bodyIds(), recorded.packet(), firstMismatchStep, noOps);
        boolean manualMatches = expectedAtMismatch.equals(manualAtMismatch);
        StepFingerprint replayPostRestore = runReplayFingerprint(config, recorded.packet(), recorded.bodyIds(), 0, noOps);
        StepFingerprint manualPostRestore = runScriptedFingerprint(config, recorded.bodyIds(), recorded.packet(), 0, noOps);
        StepFingerprint replayStep1 = runReplayFingerprint(config, recorded.packet(), recorded.bodyIds(), 1, noOps);
        StepFingerprint manualStep1 = runScriptedFingerprint(config, recorded.bodyIds(), recorded.packet(), 1, noOps);
        StepFingerprint replayStep10 = runReplayFingerprint(config, recorded.packet(), recorded.bodyIds(), 10, noOps);
        StepFingerprint manualStep10 = runScriptedFingerprint(config, recorded.bodyIds(), recorded.packet(), 10, noOps);
        SnapshotProbeDiff snapshotProbeDiff = runSnapshotSideEffectProbe(config, recorded.packet());

        System.out.println("First failing checkpoint step: " + failingCheckpoint);
        System.out.println("First mismatch step: " + firstMismatchStep);
        System.out.println("Manual rerun matches expected: " + manualMatches);
        System.out.println("mismatchHashes referenceSameWorld=" + expectedAtMismatch
            + " replayFreshWorld=" + replayAtMismatch
            + " manualFreshWorld=" + manualAtMismatch);
        System.out.println("postRestore replay=" + replayPostRestore.compact() + " manual=" + manualPostRestore.compact());
        System.out.println("postStep1 replay=" + replayStep1.compact() + " manual=" + manualStep1.compact());
        System.out.println("postStep10 replay=" + replayStep10.compact() + " manual=" + manualStep10.compact());
        System.out.println("snapshotSideEffect firstProbeMismatchStep=" + snapshotProbeDiff.firstMismatchStep());
        if (snapshotProbeDiff.firstMismatchStep() > 0) {
            System.out.println("snapshotSideEffect expected(hashEveryStep)="
                + snapshotProbeDiff.everyStepHashes().get(snapshotProbeDiff.firstMismatchStep())
                + " probeOnly=" + snapshotProbeDiff.probeOnlyHashes().get(snapshotProbeDiff.firstMismatchStep()));
        }

        assertFalse(replayResult.success(), "Expected strict Jolt replay mismatch in experimental drift locator");
        assertTrue(firstMismatchStep > 0, "First mismatch step should be positive");
    }

    private static RecordedScenario recordScenario(
        PhysicsWorldConfig config,
        int steps,
        int checkpointEverySteps,
        int hashLimit,
        boolean noOps
    ) {
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            PhysicsReplayRecorder recorder = ReplayTestSupport.newRecorder(world, PhysicsBackend.JOLT, config);
            recorder.setCheckpointEverySteps(checkpointEverySteps);
            List<RigidBodyHandle> spawned = ReplayTestSupport.setupSimpleScene(recorder);
            List<Integer> bodyIds = spawned.stream().map(ReplayTestSupport::bodyId).toList();
            recorder.captureInitialSnapshot();

            Map<Integer, String> referenceHashes = new HashMap<>();
            byte[] initial = recorder.snapshot();
            recorder.restore(initial);
            var resolver = ReplayResolvers.forWorld(PhysicsBackend.JOLT, recorder.delegateWorld());
            List<RigidBodyHandle> handles = bodyIds.stream().map(resolver::rigidBody).toList();

            for (int step = 0; step < steps; step++) {
                if (!noOps) {
                    applyScriptedOps(recorder, handles, step);
                }
                recorder.step(config.fixedTimeStep());
                int completedStep = step + 1;
                if (completedStep <= hashLimit) {
                    referenceHashes.put(completedStep, ReplayHash.sha256Hex(recorder.snapshot()));
                }
            }

            return new RecordedScenario(
                ReplayTestSupport.withValidationMode(recorder.buildPacket(), ReplayValidationMode.STRICT),
                bodyIds,
                referenceHashes
            );
        } finally {
            world.destroy();
        }
    }

    private static int findFirstMismatchStep(
        PhysicsWorldConfig config,
        org.dynamisphysics.test.replay.ReproPacket packet,
        Map<Integer, String> referenceHashes,
        int checkpointStep,
        boolean noOps
    ) {
        int low = 1;
        int high = checkpointStep;
        while (low < high) {
            int mid = low + ((high - low) / 2);
            String expected = requiredHash(referenceHashes, mid);
            String actual = runReplayHashAtStep(config, packet, mid, noOps);
            if (expected.equals(actual)) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        return low;
    }

    private static String runReplayHashAtStep(
        PhysicsWorldConfig config,
        org.dynamisphysics.test.replay.ReproPacket packet,
        int targetStep,
        boolean noOps
    ) {
        PhysicsWorld replayWorld = PhysicsWorldFactory.create(config);
        try {
            replayWorld.restore(Base64.getDecoder().decode(packet.initialSnapshotB64()));
            var resolver = ReplayResolvers.forWorld(PhysicsBackend.JOLT, replayWorld);
            Map<Integer, List<ReplayOp>> opsByStep = toOpsByStep(packet.inputs());
            for (int step = 0; step < targetStep; step++) {
                if (!noOps) {
                    List<ReplayOp> ops = opsByStep.get(step);
                    if (ops != null) {
                        for (ReplayOp op : ops) {
                            applyReplayOp(replayWorld, resolver, op);
                        }
                    }
                }
                replayWorld.step(packet.worldConfig().fixedTimeStep());
            }
            return ReplayHash.sha256Hex(replayWorld.snapshot());
        } finally {
            replayWorld.destroy();
        }
    }

    private static StepFingerprint runReplayFingerprint(
        PhysicsWorldConfig config,
        org.dynamisphysics.test.replay.ReproPacket packet,
        List<Integer> bodyIds,
        int targetStep,
        boolean noOps
    ) {
        PhysicsWorld replayWorld = PhysicsWorldFactory.create(config);
        try {
            replayWorld.restore(Base64.getDecoder().decode(packet.initialSnapshotB64()));
            var resolver = ReplayResolvers.forWorld(PhysicsBackend.JOLT, replayWorld);
            Map<Integer, List<ReplayOp>> opsByStep = toOpsByStep(packet.inputs());
            for (int step = 0; step < targetStep; step++) {
                if (!noOps) {
                    List<ReplayOp> ops = opsByStep.get(step);
                    if (ops != null) {
                        for (ReplayOp op : ops) {
                            applyReplayOp(replayWorld, resolver, op);
                        }
                    }
                }
                replayWorld.step(packet.worldConfig().fixedTimeStep());
            }
            return fingerprintOf(replayWorld, resolver, bodyIds, targetStep);
        } finally {
            replayWorld.destroy();
        }
    }

    private static String runScriptedReferenceHash(
        PhysicsWorldConfig config,
        List<Integer> bodyIds,
        org.dynamisphysics.test.replay.ReproPacket packet,
        int targetStep,
        boolean noOps
    ) {
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            world.restore(Base64.getDecoder().decode(packet.initialSnapshotB64()));
            var resolver = ReplayResolvers.forWorld(PhysicsBackend.JOLT, world);
            List<RigidBodyHandle> handles = bodyIds.stream().map(resolver::rigidBody).toList();
            for (int step = 0; step < targetStep; step++) {
                if (!noOps) {
                    applyScriptedOps(world, handles, step);
                }
                world.step(packet.worldConfig().fixedTimeStep());
            }
            return ReplayHash.sha256Hex(world.snapshot());
        } finally {
            world.destroy();
        }
    }

    private static StepFingerprint runScriptedFingerprint(
        PhysicsWorldConfig config,
        List<Integer> bodyIds,
        org.dynamisphysics.test.replay.ReproPacket packet,
        int targetStep,
        boolean noOps
    ) {
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            world.restore(Base64.getDecoder().decode(packet.initialSnapshotB64()));
            var resolver = ReplayResolvers.forWorld(PhysicsBackend.JOLT, world);
            List<RigidBodyHandle> handles = bodyIds.stream().map(resolver::rigidBody).toList();
            for (int step = 0; step < targetStep; step++) {
                if (!noOps) {
                    applyScriptedOps(world, handles, step);
                }
                world.step(packet.worldConfig().fixedTimeStep());
            }
            return fingerprintOf(world, resolver, bodyIds, targetStep);
        } finally {
            world.destroy();
        }
    }

    private static String requiredHash(Map<Integer, String> hashes, int step) {
        String hash = hashes.get(step);
        if (hash == null) {
            throw new IllegalStateException("No reference hash recorded for step " + step);
        }
        return hash;
    }

    private static Map<Integer, List<ReplayOp>> toOpsByStep(List<ReplayInputFrame> frames) {
        Map<Integer, List<ReplayOp>> out = new HashMap<>();
        for (ReplayInputFrame frame : frames) {
            out.put(frame.step(), frame.ops());
        }
        return out;
    }

    private static void applyReplayOp(PhysicsWorld world, PhysicsReplayRunner.ReplayHandleResolver resolver, ReplayOp op) {
        if (op instanceof ReplayOp.ApplyImpulseOp impulse) {
            world.applyImpulse(
                resolver.rigidBody(impulse.rigidBodyId()),
                impulse.impulse().toVector3f(),
                impulse.worldPoint().toVector3f()
            );
            return;
        }
        if (op instanceof ReplayOp.ApplyForceOp force) {
            world.applyForce(
                resolver.rigidBody(force.rigidBodyId()),
                force.force().toVector3f(),
                force.worldPoint().toVector3f()
            );
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
            world.teleport(
                resolver.rigidBody(teleport.rigidBodyId()),
                teleport.position().toVector3f(),
                teleport.orientation().toQuaternionf()
            );
            return;
        }
        throw new UnsupportedOperationException("Experimental strict Jolt replay only supports rigid body ops, got " + op.getClass().getSimpleName());
    }

    private static void applyScriptedOps(PhysicsWorld world, List<RigidBodyHandle> handles, int step) {
        if (step == 10) {
            world.applyImpulse(handles.get(0), new Vector3f(2f, 0f, 0f), new Vector3f(0f, 2f, 0f));
        }
        if (step == 30) {
            world.applyImpulse(handles.get(1), new Vector3f(0f, 0.5f, 0.2f), new Vector3f(0f, 2.2f, 0f));
        }
        if (step == 60) {
            world.applyTorque(handles.get(2), new Vector3f(0f, 0.2f, 0f));
        }
        if (step == 120) {
            world.applyForce(handles.get(3), new Vector3f(0.5f, 0f, 0f), new Vector3f(0f, 2.1f, 0f));
        }
    }

    private static StepFingerprint fingerprintOf(
        PhysicsWorld world,
        PhysicsReplayRunner.ReplayHandleResolver resolver,
        List<Integer> bodyIds,
        int step
    ) {
        var stats = world.getStats();
        long bodyChecksum = 1469598103934665603L;
        for (int id : bodyIds) {
            var state = world.getBodyState(resolver.rigidBody(id));
            bodyChecksum = mix(bodyChecksum, id);
            bodyChecksum = mix(bodyChecksum, quant(state.position().x));
            bodyChecksum = mix(bodyChecksum, quant(state.position().y));
            bodyChecksum = mix(bodyChecksum, quant(state.position().z));
            Quaternionf q = state.orientation();
            bodyChecksum = mix(bodyChecksum, quant(q.x));
            bodyChecksum = mix(bodyChecksum, quant(q.y));
            bodyChecksum = mix(bodyChecksum, quant(q.z));
            bodyChecksum = mix(bodyChecksum, quant(q.w));
            bodyChecksum = mix(bodyChecksum, quant(state.linearVelocity().x));
            bodyChecksum = mix(bodyChecksum, quant(state.linearVelocity().y));
            bodyChecksum = mix(bodyChecksum, quant(state.linearVelocity().z));
            bodyChecksum = mix(bodyChecksum, quant(state.angularVelocity().x));
            bodyChecksum = mix(bodyChecksum, quant(state.angularVelocity().y));
            bodyChecksum = mix(bodyChecksum, quant(state.angularVelocity().z));
            bodyChecksum = mix(bodyChecksum, state.sleeping() ? 1 : 0);
        }
        return new StepFingerprint(
            step,
            ReplayHash.sha256Hex(world.snapshot()),
            stats.bodyCount(),
            stats.activeBodyCount(),
            stats.sleepingBodyCount(),
            stats.islandCount(),
            bodyChecksum
        );
    }

    private static long mix(long seed, int value) {
        long x = seed ^ value;
        return x * 1099511628211L;
    }

    private static int quant(float value) {
        return Math.round(value * 1_000_000f);
    }

    private static SnapshotProbeDiff runSnapshotSideEffectProbe(
        PhysicsWorldConfig config,
        org.dynamisphysics.test.replay.ReproPacket packet
    ) {
        Set<Integer> probeSteps = parseProbeSteps(System.getProperty("physics.replay.probeSteps", "1,10,20,30,36,40,50"));
        Map<Integer, String> everyStep = runNoOpProbeHashes(config, packet, probeSteps, true);
        Map<Integer, String> probeOnly = runNoOpProbeHashes(config, packet, probeSteps, false);
        int firstMismatch = 0;
        for (int step : probeSteps) {
            String a = everyStep.get(step);
            String b = probeOnly.get(step);
            if (!a.equals(b)) {
                firstMismatch = step;
                break;
            }
        }
        return new SnapshotProbeDiff(everyStep, probeOnly, firstMismatch);
    }

    private static Map<Integer, String> runNoOpProbeHashes(
        PhysicsWorldConfig config,
        org.dynamisphysics.test.replay.ReproPacket packet,
        Set<Integer> probeSteps,
        boolean snapshotEveryStep
    ) {
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            world.restore(Base64.getDecoder().decode(packet.initialSnapshotB64()));
            int maxStep = probeSteps.stream().mapToInt(Integer::intValue).max().orElse(0);
            Map<Integer, String> out = new LinkedHashMap<>();
            for (int step = 1; step <= maxStep; step++) {
                world.step(packet.worldConfig().fixedTimeStep());
                if (snapshotEveryStep) {
                    String hash = ReplayHash.sha256Hex(world.snapshot());
                    if (probeSteps.contains(step)) {
                        out.put(step, hash);
                    }
                } else if (probeSteps.contains(step)) {
                    out.put(step, ReplayHash.sha256Hex(world.snapshot()));
                }
            }
            return out;
        } finally {
            world.destroy();
        }
    }

    private static Set<Integer> parseProbeSteps(String raw) {
        Set<Integer> steps = new TreeSet<>();
        Arrays.stream(raw.split(","))
            .map(String::trim)
            .filter(s -> !s.isEmpty())
            .forEach(s -> steps.add(Integer.parseInt(s)));
        return steps;
    }

    private record RecordedScenario(
        org.dynamisphysics.test.replay.ReproPacket packet,
        List<Integer> bodyIds,
        Map<Integer, String> referenceHashes
    ) {
    }

    private record StepFingerprint(
        int step,
        String snapshotHash,
        int bodyCount,
        int activeBodyCount,
        int sleepingBodyCount,
        int islandCount,
        long bodyChecksum
    ) {
        String compact() {
            return "step=" + step
                + " hash=" + snapshotHash
                + " bodies=" + bodyCount
                + " active=" + activeBodyCount
                + " sleeping=" + sleepingBodyCount
                + " islands=" + islandCount
                + " checksum=" + Long.toUnsignedString(bodyChecksum);
        }
    }

    private record SnapshotProbeDiff(
        Map<Integer, String> everyStepHashes,
        Map<Integer, String> probeOnlyHashes,
        int firstMismatchStep
    ) {
    }
}
