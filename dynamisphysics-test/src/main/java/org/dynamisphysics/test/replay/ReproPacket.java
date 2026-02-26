package org.dynamisphysics.test.replay;

import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.ResolvedTuning;

import java.util.List;
import java.util.Map;

public record ReproPacket(
    String magic,
    int formatVersion,
    String createdUtc,
    String engineVersion,
    PhysicsBackend backend,
    ReproTuning tuning,
    ReproWorldConfig worldConfig,
    ReproScene scene,
    ReplayValidationMode validationMode,
    ReplayInvariants invariants,
    long seed,
    String initialSnapshotB64,
    List<ReplayInputFrame> inputs,
    List<ReplayCheckpoint> checkpoints
) {
    public static final int FORMAT_VERSION = ReplayPacketSchema.VERSION;
    public static final String MAGIC = ReplayPacketSchema.MAGIC;
    public static final String SCHEMA_FINGERPRINT = ReplayPacketSchema.FINGERPRINT;

    public static ReproPacket of(
        String engineVersion,
        PhysicsBackend backend,
        ResolvedTuning tuning,
        float fixedTimeStep,
        int maxSubSteps,
        ReproScene scene,
        ReplayValidationMode validationMode,
        ReplayInvariants invariants,
        long seed,
        String initialSnapshotB64,
        List<ReplayInputFrame> inputs,
        List<ReplayCheckpoint> checkpoints
    ) {
        return new ReproPacket(
            MAGIC,
            FORMAT_VERSION,
            java.time.Instant.now().toString(),
            engineVersion,
            backend,
            ReproTuning.fromResolved(tuning),
            new ReproWorldConfig(fixedTimeStep, maxSubSteps),
            scene,
            validationMode,
            invariants,
            seed,
            initialSnapshotB64,
            List.copyOf(inputs),
            List.copyOf(checkpoints)
        );
    }

    public record ReproTuning(
        String profile,
        boolean deterministic,
        int threads,
        String allocatorMode,
        int allocatorMb,
        int solverIterations
    ) {
        static ReproTuning fromResolved(ResolvedTuning tuning) {
            return new ReproTuning(
                tuning.profile().name(),
                tuning.deterministic(),
                tuning.threads(),
                tuning.allocatorMode().name(),
                Math.max(1, tuning.allocatorBytes() / (1024 * 1024)),
                tuning.solverIterations()
            );
        }
    }

    public record ReproWorldConfig(float fixedTimeStep, int maxSubSteps) {
    }

    public record ReproScene(String name, Map<String, Object> params) {
        public ReproScene {
            params = params == null ? Map.of() : Map.copyOf(params);
        }
    }
}
