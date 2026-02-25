package org.dynamisphysics.jolt.integration;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.jolt.JoltBackendRegistrar;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.vectrix.core.Matrix4f;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.parity.integration", matches = "true")
class SnapshotMidMotionParityTest {
    private static final int PRE_STEPS = 180;
    private static final int POST_STEPS = 180;

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void backendLocalRestoreMatchesContinuous(PhysicsBackend backend) {
        RunResult continuous = runScenario(backend, false);
        RunResult restored = runScenario(backend, true);

        assertEquals(continuous.dynamicCount(), restored.dynamicCount(), "dynamic count mismatch");
        assertEquals(continuous.dynamicIds(), restored.dynamicIds(), "dynamic id set mismatch");

        float avgYEps = backend == PhysicsBackend.ODE4J ? 0.05f : 0.08f;
        float avgSpeedEps = backend == PhysicsBackend.ODE4J ? 0.2f : 0.3f;
        assertTrue(java.lang.Math.abs(continuous.avgY() - restored.avgY()) <= avgYEps,
            "avgY mismatch " + continuous.avgY() + " vs " + restored.avgY());
        assertTrue(java.lang.Math.abs(continuous.avgSpeed() - restored.avgSpeed()) <= avgSpeedEps,
            "avgSpeed mismatch " + continuous.avgSpeed() + " vs " + restored.avgSpeed());
        assertTrue(restored.minY() > -5f, "unstable minY: " + restored.minY());
        assertTrue(restored.eventCount() > 0, "expected post-restore events");
    }

    private static RunResult runScenario(PhysicsBackend backend, boolean restoreAtMid) {
        PhysicsWorld world = IntegrationParityScenes.newWorld(backend, true);
        try {
            IntegrationParityScenes.spawnGroundBox(world);
            List<RigidBodyHandle> tracked = new ArrayList<>();
            for (int i = 0; i < 8; i++) {
                tracked.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.3f), 1f)
                    .worldTransform(new Matrix4f().identity().translation(-2f + i * 0.7f, 2.2f + i * 0.2f, 0f))
                    .build()));
            }
            IntegrationParityScenes.ConstraintModule spring = IntegrationParityScenes.addSpringModule(world, -3f);
            tracked.add(spring.bodyA());
            tracked.add(spring.bodyB());

            for (int i = 0; i < PRE_STEPS; i++) {
                if (i % 60 == 0) {
                    for (int b = 0; b < tracked.size(); b += 3) {
                        world.applyImpulse(tracked.get(b), new org.vectrix.core.Vector3f(1f, 0f, 0f), new org.vectrix.core.Vector3f());
                    }
                }
                world.step(IntegrationParityScenes.DT, 1);
                world.drainEvents();
            }
            byte[] snapshot = world.snapshot();
            if (restoreAtMid) {
                world.restore(snapshot);
            }

            List<PhysicsEvent> events = new ArrayList<>();
            for (int i = 0; i < POST_STEPS; i++) {
                world.step(IntegrationParityScenes.DT, 1);
                events.addAll(world.drainEvents());
            }

            List<IntegrationParityScenes.SnapshotState> states =
                IntegrationParityScenes.decodeDynamicStates(backend, world.snapshot());
            float minY = Float.POSITIVE_INFINITY;
            float sumY = 0f;
            float sumSpeed = 0f;
            for (IntegrationParityScenes.SnapshotState s : states) {
                minY = java.lang.Math.min(minY, s.position().y());
                sumY += s.position().y();
                sumSpeed += s.linearVelocity().length();
            }
            int n = java.lang.Math.max(1, states.size());
            return new RunResult(
                states.size(),
                states.stream().map(IntegrationParityScenes.SnapshotState::bodyId).collect(Collectors.toSet()),
                sumY / n,
                sumSpeed / n,
                minY,
                events.size()
            );
        } finally {
            world.destroy();
        }
    }

    private record RunResult(
        int dynamicCount,
        Set<Integer> dynamicIds,
        float avgY,
        float avgSpeed,
        float minY,
        int eventCount
    ) {
    }
}
