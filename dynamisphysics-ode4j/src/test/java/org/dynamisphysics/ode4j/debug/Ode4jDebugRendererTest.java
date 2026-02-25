package org.dynamisphysics.ode4j.debug;

import org.dynamisgpu.api.gpu.BoundsBuffer;
import org.dynamisgpu.api.gpu.StagingScheduler;
import org.dynamisgpu.api.gpu.VkCommandBuffer;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsDebugConfig;
import org.dynamisphysics.ode4j.Ode4jPhysicsWorld;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.dynamiscollision.shapes.CollisionShape;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jDebugRendererTest {
    private Ode4jPhysicsWorld world;

    @BeforeAll
    static void registerBackend() {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    @BeforeEach
    void setUp() {
        world = (Ode4jPhysicsWorld) PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
    }

    @AfterEach
    void tearDown() {
        world.destroy();
    }

    @Test
    void rendererCreatesWithoutError() {
        assertDoesNotThrow(() -> new Ode4jDebugRenderer(1024));
    }

    @Test
    void updateWithNoBodiesProducesZeroVertices() {
        Ode4jDebugRenderer renderer = new Ode4jDebugRenderer(1024);
        renderer.update(world, new PhysicsDebugConfig(true, false, false, false, false, false, false, true));
        assertEquals(0, renderer.lastVertexCount());
    }

    @Test
    void updateWithBodiesProducesNonZeroVerticesWhenShowShapes() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(1f, 1f, 1f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 2f, 0f))
            .build());

        Ode4jDebugRenderer renderer = new Ode4jDebugRenderer(1024);
        renderer.update(world, PhysicsDebugConfig.shapesOnly());
        assertTrue(renderer.lastVertexCount() > 0);
    }

    @Test
    void updateAddsContactPointsWhenConfigured() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.planeY(), 0f)
            .mode(BodyMode.STATIC)
            .build());
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 0.4f, 0f))
            .build());
        world.step(1f / 60f);

        Ode4jDebugRenderer renderer = new Ode4jDebugRenderer(2048);
        renderer.update(world, new PhysicsDebugConfig(false, true, false, false, false, false, false, false));
        assertTrue(renderer.lastVertexCount() > 0);
    }

    @Test
    void updateAddsVelocityVectorsWhenConfigured() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .linearVelocity(new Vector3f(3f, 0f, 0f))
            .worldTransform(new Matrix4f().translation(0f, 3f, 0f))
            .build());

        Ode4jDebugRenderer renderer = new Ode4jDebugRenderer(1024);
        renderer.update(world, new PhysicsDebugConfig(false, false, false, true, false, false, false, false));
        assertTrue(renderer.lastVertexCount() > 0);
    }

    @Test
    void recordDoesNotThrowWithValidCommandBuffer() {
        FakeBoundsBuffer bounds = new FakeBoundsBuffer();
        FakeStaging staging = new FakeStaging();
        Ode4jDebugRenderer renderer = new Ode4jDebugRenderer(128, bounds, staging);

        renderer.record(new FakeCommandBuffer());
        assertTrue(staging.flushCalled);
    }

    @Test
    void destroyReleasesGpuBuffer() {
        FakeBoundsBuffer bounds = new FakeBoundsBuffer();
        FakeStaging staging = new FakeStaging();
        Ode4jDebugRenderer renderer = new Ode4jDebugRenderer(128, bounds, staging);

        renderer.destroy();
        assertTrue(bounds.destroyed);
        assertTrue(staging.destroyed);
    }

    private static final class FakeBoundsBuffer implements BoundsBuffer {
        private boolean destroyed;

        @Override public void writeBounds(int i, float x, float y, float z, float r) {}
        @Override public void flush() {}
        @Override public long bufferHandle() { return 42L; }
        @Override public void destroy() { destroyed = true; }
    }

    private static final class FakeStaging implements StagingScheduler {
        private boolean flushCalled;
        private boolean destroyed;

        @Override public void markDirty(long handle, long offset, long size) {}
        @Override public void flush(VkCommandBuffer cmd) { flushCalled = true; }
        @Override public void reset() {}
        @Override public void destroy() { destroyed = true; }
    }

    private static final class FakeCommandBuffer implements VkCommandBuffer {
        @Override public long handle() { return 7L; }
    }
}
