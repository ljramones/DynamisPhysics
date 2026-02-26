package org.dynamisphysics.jolt;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsTuningResolver;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.test.replay.PhysicsReplayRecorder;
import org.dynamisphysics.test.replay.PhysicsReplayRunner;
import org.dynamisphysics.test.replay.ReproPacket;
import org.dynamisphysics.test.replay.ReplayValidationMode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.vectrix.core.Vector3f;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.replay.tests", matches = "true")
class ReplayStrictRoundTripTest {
    @BeforeAll
    static void registerBackends() {
        ReplayTestSupport.registerBackends();
    }

    @ParameterizedTest
    @EnumSource(PhysicsBackend.class)
    void strictRoundTripMatchesCheckpoints(PhysicsBackend backend) {
        PhysicsWorldConfig config = ReplayTestSupport.deterministicConfig(backend);
        ReplayBackendAssertions.requireStrict(backend, PhysicsTuningResolver.resolve(config));
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            PhysicsReplayRecorder recorder = ReplayTestSupport.newRecorder(world, backend, config);
            recorder.setCheckpointEverySteps(60);
            List<RigidBodyHandle> spawned = ReplayTestSupport.setupSimpleScene(recorder);
            List<Integer> ids = spawned.stream().map(ReplayTestSupport::bodyId).toList();
            recorder.captureInitialSnapshot();
            byte[] initial = recorder.snapshot();
            recorder.restore(initial);
            var recordResolver = ReplayResolvers.forWorld(backend, recorder.delegateWorld());
            List<RigidBodyHandle> replayHandles = ids.stream().map(recordResolver::rigidBody).toList();

            for (int i = 0; i < 240; i++) {
                if (i == 10) {
                    recorder.applyImpulse(replayHandles.get(0), new Vector3f(2f, 0f, 0f), new Vector3f(0f, 2f, 0f));
                }
                recorder.step(config.fixedTimeStep());
            }

            ReproPacket packet = ReplayTestSupport.withValidationMode(recorder.buildPacket(), ReplayValidationMode.STRICT);
            packet = ReplayTestSupport.canonicalizeCheckpointsFromFreshRestore(backend, config, packet);

            PhysicsReplayRunner.ReplayResult direct = ReplayTestSupport.runPacket(backend, config, packet);
            assertTrue(direct.success(), "direct packet replay failed: " + direct.message());

            ReproPacket parsed = ReplayTestSupport.roundTripJson(packet);
            PhysicsReplayRunner.ReplayResult parsedResult = ReplayTestSupport.runPacket(backend, config, parsed);
            assertTrue(parsedResult.success(), parsedResult.message());
        } finally {
            world.destroy();
        }
    }
}
