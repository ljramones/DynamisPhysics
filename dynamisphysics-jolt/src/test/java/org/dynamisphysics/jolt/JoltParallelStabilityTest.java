package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.jolt.stress", matches = "true")
class JoltParallelStabilityTest {
    private static final float DT = 1f / 60f;

    @BeforeAll
    static void registerBackend() {
        new JoltBackendRegistrar();
    }

    @Test
    void manySpheresPileRemainsStable() {
        int bodyCount = Integer.getInteger("physics.jolt.stress.size", 2000);
        PhysicsWorld world = createStressWorld();
        try {
            spawnGround(world);
            List<RigidBodyHandle> bodies = spawnPile(world, bodyCount);

            float avgSpeedAt60 = runAndMeasure(world, bodies, 60).averageSpeed;
            Metrics end = runAndMeasure(world, bodies, 540);

            assertTrue(end.allFinite, "Detected NaN/INF body state");
            assertTrue(end.minY > -5f, "Bodies fell through floor. minY=" + end.minY);
            assertTrue(end.averageSpeed <= avgSpeedAt60 + 1e-6f,
                "Expected settling. speed@60=" + avgSpeedAt60 + " speed@end=" + end.averageSpeed);

            System.out.printf(
                "JOLT-STRESS pile bodies=%d avgStepMs=%.3f minY=%.3f events=%d%n",
                bodyCount, end.averageStepMs, end.minY, end.totalEvents
            );
        } finally {
            world.destroy();
        }
    }

    @Test
    void manyIslandsRemainStable() {
        int islands = Integer.getInteger("physics.jolt.stress.islands", 120);
        int spheresPerIsland = 10;
        int bodyCount = islands * spheresPerIsland;

        PhysicsWorld world = createStressWorld();
        try {
            spawnGround(world);
            List<RigidBodyHandle> handles = spawnIslands(world, islands, spheresPerIsland);
            Metrics metrics = runAndMeasure(world, handles, 300);

            assertTrue(metrics.allFinite, "Detected NaN/INF body state");
            assertTrue(metrics.minY > -5f, "Bodies fell through floor. minY=" + metrics.minY);

            System.out.printf(
                "JOLT-STRESS islands=%d bodies=%d avgStepMs=%.3f minY=%.3f events=%d%n",
                islands, bodyCount, metrics.averageStepMs, metrics.minY, metrics.totalEvents
            );
        } finally {
            world.destroy();
        }
    }

    @Test
    void contactEventDrainRemainsStableUnderLoad() {
        int bodyCount = Integer.getInteger("physics.jolt.stress.size", 2000);
        PhysicsWorld world = createStressWorld();
        try {
            spawnGround(world);
            List<RigidBodyHandle> bodies = spawnPile(world, bodyCount);
            Metrics metrics = runAndMeasure(world, bodies, 240);

            assertTrue(metrics.totalContactEvents > 0, "Expected contact events under load");
            assertTrue(metrics.allFinite, "Detected NaN/INF body state");

            System.out.printf(
                "JOLT-STRESS events bodies=%d avgStepMs=%.3f contacts=%d totalEvents=%d%n",
                bodyCount, metrics.averageStepMs, metrics.totalContactEvents, metrics.totalEvents
            );
        } finally {
            world.destroy();
        }
    }

    private static PhysicsWorld createStressWorld() {
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.JOLT,
            new Vector3f(0f, -9.81f, 0f),
            DT,
            1,
            10,
            131_072,
            32_768,
            BroadphaseType.BVH,
            false
        );
        return PhysicsWorldFactory.create(cfg);
    }

    private static void spawnGround(PhysicsWorld world) {
        world.spawnRigidBody(
            RigidBodyConfig.builder(CollisionShape.box(200f, 1f, 200f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().identity().translation(0f, -1f, 0f))
                .build()
        );
    }

    private static List<RigidBodyHandle> spawnPile(PhysicsWorld world, int bodyCount) {
        int perLayer = Math.max(1, (int) java.lang.Math.ceil(java.lang.Math.sqrt(bodyCount / 2.0)));
        List<RigidBodyHandle> out = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            int layer = i / (perLayer * perLayer);
            int idx = i % (perLayer * perLayer);
            int gx = idx % perLayer;
            int gz = idx / perLayer;
            float x = (gx - perLayer * 0.5f) * 0.6f;
            float z = (gz - perLayer * 0.5f) * 0.6f;
            float y = 0.35f + layer * 0.6f;
            out.add(world.spawnRigidBody(
                RigidBodyConfig.builder(CollisionShape.sphere(0.25f), 1f)
                    .worldTransform(new Matrix4f().identity().translation(x, y, z))
                    .build()
            ));
        }
        return out;
    }

    private static List<RigidBodyHandle> spawnIslands(PhysicsWorld world, int islands, int perIsland) {
        List<RigidBodyHandle> out = new ArrayList<>(islands * perIsland);
        int columns = Math.max(1, (int) java.lang.Math.ceil(java.lang.Math.sqrt(islands)));
        for (int i = 0; i < islands; i++) {
            int ix = i % columns;
            int iz = i / columns;
            float baseX = (ix - columns * 0.5f) * 8f;
            float baseZ = (iz - columns * 0.5f) * 8f;
            for (int j = 0; j < perIsland; j++) {
                float y = 0.35f + j * 0.6f;
                out.add(world.spawnRigidBody(
                    RigidBodyConfig.builder(CollisionShape.sphere(0.25f), 1f)
                        .worldTransform(new Matrix4f().identity().translation(baseX, y, baseZ))
                        .build()
                ));
            }
        }
        return out;
    }

    private static Metrics runAndMeasure(PhysicsWorld world, List<RigidBodyHandle> bodies, int steps) {
        long startNanos = System.nanoTime();
        int totalEvents = 0;
        int contactEvents = 0;
        for (int i = 0; i < steps; i++) {
            world.step(DT, 1);
            List<PhysicsEvent> events = world.drainEvents();
            totalEvents += events.size();
            for (PhysicsEvent event : events) {
                if (event instanceof ContactEvent) {
                    contactEvents++;
                }
            }
        }
        float elapsedMs = (System.nanoTime() - startNanos) / 1_000_000f;
        float avgStepMs = elapsedMs / steps;

        boolean finite = true;
        float minY = Float.POSITIVE_INFINITY;
        float speedSum = 0f;
        for (RigidBodyHandle body : bodies) {
            BodyState state = world.getBodyState(body);
            finite &= isFinite(state.position()) && isFinite(state.linearVelocity()) && isFinite(state.angularVelocity());
            minY = java.lang.Math.min(minY, state.position().y());
            speedSum += state.linearVelocity().length();
        }
        float averageSpeed = speedSum / bodies.size();
        return new Metrics(finite, minY, averageSpeed, avgStepMs, totalEvents, contactEvents);
    }

    private static boolean isFinite(Vector3f v) {
        return Float.isFinite(v.x()) && Float.isFinite(v.y()) && Float.isFinite(v.z());
    }

    private record Metrics(
        boolean allFinite,
        float minY,
        float averageSpeed,
        float averageStepMs,
        int totalEvents,
        int totalContactEvents
    ) {
    }
}
