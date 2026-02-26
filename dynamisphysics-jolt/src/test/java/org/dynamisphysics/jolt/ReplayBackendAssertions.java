package org.dynamisphysics.jolt;

import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.ResolvedTuning;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class ReplayBackendAssertions {
    private static final Map<PhysicsBackend, Capabilities> CAPS = Map.of(
        PhysicsBackend.ODE4J, new Capabilities(
            true,  // supportsStrict
            true,  // supportsBehavioural
            true,  // requiresFreshCanonicalReference
            false, // requiresThreads1ForStrict
            true   // allowsPerfAllocatorInBehavioural
        ),
        PhysicsBackend.JOLT, new Capabilities(
            true,  // supportsStrict
            true,  // supportsBehavioural
            true,  // requiresFreshCanonicalReference
            true,  // requiresThreads1ForStrict
            true   // allowsPerfAllocatorInBehavioural
        )
    );

    private ReplayBackendAssertions() {
    }

    static void requireStrict(PhysicsBackend backend, ResolvedTuning tuning) {
        Capabilities caps = CAPS.get(backend);
        assertTrue(caps.supportsStrict(), backend + " does not support STRICT replay");
        assertTrue(caps.requiresFreshCanonicalReference(), backend + " STRICT replay must use canonical fresh-restore references");
        if (caps.requiresThreads1ForStrict()) {
            assertEquals(1, tuning.threads(), backend + " STRICT replay requires threads=1");
        }
    }

    static void requireBehavioural(PhysicsBackend backend, ResolvedTuning tuning) {
        Capabilities caps = CAPS.get(backend);
        assertTrue(caps.supportsBehavioural(), backend + " does not support BEHAVIOURAL replay");
        if (!caps.allowsPerfAllocatorInBehavioural()) {
            assertTrue(tuning.deterministic(), backend + " behavioural replay requires deterministic tuning");
        }
    }

    private record Capabilities(
        boolean supportsStrict,
        boolean supportsBehavioural,
        boolean requiresFreshCanonicalReference,
        boolean requiresThreads1ForStrict,
        boolean allowsPerfAllocatorInBehavioural
    ) {
    }
}
