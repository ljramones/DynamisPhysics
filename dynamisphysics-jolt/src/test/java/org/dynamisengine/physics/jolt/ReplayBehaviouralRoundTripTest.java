package org.dynamisengine.physics.jolt;

import org.dynamisengine.physics.api.PhysicsWorldFactory;
import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.config.PhysicsBackend;
import org.dynamisengine.physics.api.config.PhysicsTuningResolver;
import org.dynamisengine.physics.api.config.PhysicsWorldConfig;
import org.dynamisengine.physics.api.world.PhysicsWorld;
import org.dynamisengine.physics.test.replay.PhysicsReplayRecorder;
import org.dynamisengine.physics.test.replay.PhysicsReplayRunner;
import org.dynamisengine.physics.test.replay.ReproPacket;
import org.dynamisengine.physics.test.replay.ReplayValidationMode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.junit.jupiter.api.Test;
import org.dynamisengine.vectrix.core.Vector3f;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.replay.tests", matches = "true")
class ReplayBehaviouralRoundTripTest {
    @BeforeAll
    static void registerBackends() {
        ReplayTestSupport.registerBackends();
    }

    @Test
    void behaviouralRoundTripExecutesUnderJoltPerf() {
        PhysicsBackend backend = PhysicsBackend.JOLT;
        PhysicsWorldConfig config = ReplayTestSupport.perfConfig(backend);
        ReplayBackendAssertions.requireBehavioural(backend, PhysicsTuningResolver.resolve(config));
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
                if (i == 40) {
                    recorder.applyForce(replayHandles.get(1), new Vector3f(-1.5f, 0f, 0f), new Vector3f(0f, 2f, 0f));
                }
                recorder.step(config.fixedTimeStep());
            }

            ReproPacket packet = ReplayTestSupport.withValidationMode(recorder.buildPacket(), ReplayValidationMode.BEHAVIOURAL);
            PhysicsReplayRunner.ReplayResult direct = ReplayTestSupport.runPacket(backend, config, packet);
            assertTrue(direct.success(), "direct packet replay failed: " + direct.message());
            PhysicsReplayRunner.ReplayResult parsed = ReplayTestSupport.runPacket(
                backend,
                config,
                ReplayTestSupport.roundTripJson(packet)
            );
            assertTrue(parsed.success(), parsed.message());
        } finally {
            world.destroy();
        }
    }
}
