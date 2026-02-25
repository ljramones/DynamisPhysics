package org.dynamisphysics.api.config;

import java.util.Locale;

public final class PhysicsTuningResolver {
    private static final int MAX_THREADS = 64;
    private static final int MAX_SOLVER_ITERATIONS = 64;
    private static final int MAX_ALLOCATOR_MB = 1024;

    private PhysicsTuningResolver() {
    }

    public static ResolvedTuning resolve(PhysicsWorldConfig cfg) {
        PhysicsBackend backend = cfg.backend();
        PhysicsTuning tuning = cfg.tuning() != null ? cfg.tuning() : PhysicsTuning.defaults();

        PhysicsTuningProfile profile = defaultedProfile(tuning.profile());
        Mutable values = profileDefaults(profile, backend);

        // Legacy PhysicsWorldConfig fields are mapped only here to keep one source of precedence logic.
        values.deterministic = cfg.deterministic();
        values.solverIterations = cfg.solverIterations();

        // Explicit tuning fields override legacy fields.
        if (tuning.deterministic() != null) {
            values.deterministic = tuning.deterministic();
        }
        if (tuning.solverIterations() != null) {
            values.solverIterations = tuning.solverIterations();
        }
        if (tuning.threads() != null) {
            values.threads = tuning.threads();
        }
        if (tuning.allocatorMode() != null) {
            values.allocatorMode = tuning.allocatorMode();
        }
        if (tuning.allocatorMb() != null) {
            values.allocatorMb = tuning.allocatorMb();
        }

        // System properties are highest precedence.
        PhysicsTuningProfile profileOverride = parseEnumProperty("physics.profile", PhysicsTuningProfile.class);
        if (profileOverride != null) {
            profile = profileOverride;
            values = profileDefaults(profile, backend);
        }

        Boolean deterministicOverride = parseBooleanProperty("physics.deterministic");
        if (deterministicOverride != null) {
            values.deterministic = deterministicOverride;
        }
        Integer solverOverride = parseIntProperty("physics.solver.iterations");
        if (solverOverride != null) {
            values.solverIterations = solverOverride;
        }
        Integer threadOverride = parseIntProperty("jolt.threads");
        if (threadOverride != null) {
            values.threads = threadOverride;
        }
        AllocatorMode allocatorOverride = parseEnumProperty("jolt.alloc", AllocatorMode.class);
        if (allocatorOverride != null) {
            values.allocatorMode = allocatorOverride;
        }
        Integer allocatorMbOverride = parseIntProperty("jolt.alloc.mb");
        if (allocatorMbOverride != null) {
            values.allocatorMb = allocatorMbOverride;
        }

        values.solverIterations = clamp(values.solverIterations, 1, MAX_SOLVER_ITERATIONS, "physics.solver.iterations");
        values.allocatorMb = clamp(values.allocatorMb, 1, MAX_ALLOCATOR_MB, "jolt.alloc.mb");
        int allocatorBytes = values.allocatorMb * 1024 * 1024;

        int threads;
        if (backend == PhysicsBackend.JOLT) {
            int resolved = values.threads == null
                ? (values.deterministic ? 1 : Runtime.getRuntime().availableProcessors())
                : values.threads;
            threads = clamp(resolved, 1, MAX_THREADS, "jolt.threads");
        } else {
            threads = 1;
        }

        AllocatorMode allocatorMode = values.allocatorMode;
        if (backend == PhysicsBackend.JOLT && allocatorMode == AllocatorMode.SAFE) {
            allocatorMode = AllocatorMode.MALLOC;
        }

        return new ResolvedTuning(profile, values.deterministic, values.solverIterations, threads, allocatorMode, allocatorBytes);
    }

    private static Mutable profileDefaults(PhysicsTuningProfile profile, PhysicsBackend backend) {
        return switch (profile) {
            case DETERMINISTIC -> new Mutable(true, 16, 1, AllocatorMode.SAFE, 64);
            case DEFAULT -> new Mutable(true, 12, 1, AllocatorMode.SAFE, 64);
            case PERF -> new Mutable(false, 8, backend == PhysicsBackend.JOLT ? null : 1, AllocatorMode.IMPL, 128);
        };
    }

    private static PhysicsTuningProfile defaultedProfile(PhysicsTuningProfile profile) {
        return profile == null ? PhysicsTuningProfile.DEFAULT : profile;
    }

    private static <E extends Enum<E>> E parseEnumProperty(String name, Class<E> type) {
        String raw = System.getProperty(name);
        if (raw == null || raw.isBlank()) {
            return null;
        }
        try {
            return Enum.valueOf(type, raw.trim().toUpperCase(Locale.ROOT));
        } catch (IllegalArgumentException ex) {
            warn("Ignoring invalid value for " + name + ": " + raw);
            return null;
        }
    }

    private static Boolean parseBooleanProperty(String name) {
        String raw = System.getProperty(name);
        if (raw == null || raw.isBlank()) {
            return null;
        }
        return Boolean.parseBoolean(raw);
    }

    private static Integer parseIntProperty(String name) {
        String raw = System.getProperty(name);
        if (raw == null || raw.isBlank()) {
            return null;
        }
        try {
            return Integer.parseInt(raw.trim());
        } catch (NumberFormatException ex) {
            warn("Ignoring invalid integer for " + name + ": " + raw);
            return null;
        }
    }

    private static int clamp(int value, int min, int max, String key) {
        if (value < min) {
            warn("Clamped " + key + " from " + value + " to " + min);
            return min;
        }
        if (value > max) {
            warn("Clamped " + key + " from " + value + " to " + max);
            return max;
        }
        return value;
    }

    private static void warn(String message) {
        System.err.println("DYNAMIS-TUNING " + message);
    }

    private static final class Mutable {
        private boolean deterministic;
        private int solverIterations;
        private Integer threads;
        private AllocatorMode allocatorMode;
        private int allocatorMb;

        private Mutable(boolean deterministic, int solverIterations, Integer threads, AllocatorMode allocatorMode, int allocatorMb) {
            this.deterministic = deterministic;
            this.solverIterations = solverIterations;
            this.threads = threads;
            this.allocatorMode = allocatorMode;
            this.allocatorMb = allocatorMb;
        }
    }
}
