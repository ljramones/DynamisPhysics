package org.dynamisphysics.bench;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.jolt.JoltBackendRegistrar;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.vectrix.core.Matrix4f;

import java.util.ArrayList;
import java.util.List;

final class BenchSupport {
    private static volatile boolean registered;

    private BenchSupport() {
    }

    static void ensureBackendsRegistered() {
        if (registered) {
            return;
        }
        synchronized (BenchSupport.class) {
            if (registered) {
                return;
            }
            new Ode4jBackendRegistrar();
            new JoltBackendRegistrar();
            registered = true;
        }
    }

    static PhysicsWorld createWorld(PhysicsBackend backend, boolean deterministic) {
        ensureBackendsRegistered();
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            backend,
            PhysicsWorldConfig.defaults(backend).gravity(),
            1f / 60f,
            1,
            10,
            100_000,
            20_000,
            PhysicsWorldConfig.defaults(backend).broadphase(),
            deterministic
        );
        return PhysicsWorldFactory.create(cfg);
    }

    static RigidBodyHandle spawnGround(PhysicsWorld world) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(200f, 1f, 200f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .worldTransform(new Matrix4f().translation(0f, -1f, 0f))
            .build());
    }

    static List<RigidBodyHandle> spawnSphereGrid(PhysicsWorld world, int count, float radius) {
        List<RigidBodyHandle> handles = new ArrayList<>(count);
        int side = (int) java.lang.Math.ceil(java.lang.Math.sqrt(count));
        float spacing = radius * 2.25f;
        for (int i = 0; i < count; i++) {
            int x = i % side;
            int z = i / side;
            float px = (x - (side * 0.5f)) * spacing;
            float pz = (z - (side * 0.5f)) * spacing;
            float py = 4f + (i % 4) * (radius * 2.5f);
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(radius), 1f)
                .worldTransform(new Matrix4f().translation(px, py, pz))
                .material(PhysicsMaterial.DEFAULT)
                .build()));
        }
        return handles;
    }

    static List<RigidBodyHandle> spawnRaycastTargets(PhysicsWorld world, int count) {
        List<RigidBodyHandle> handles = new ArrayList<>(count);
        int side = (int) java.lang.Math.ceil(java.lang.Math.sqrt(count));
        for (int i = 0; i < count; i++) {
            int x = i % side;
            int z = i / side;
            float px = (x - side / 2f) * 2.2f;
            float pz = (z - side / 2f) * 2.2f;
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.5f, 0.5f, 0.5f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().translation(px, 0.5f, pz))
                .build()));
        }
        return handles;
    }

    static void warmStart(PhysicsWorld world, int steps) {
        for (int i = 0; i < steps; i++) {
            world.step(1f / 60f, 1);
        }
    }

    static VehicleDescriptor defaultVehicleDescriptor() {
        return VehicleDescriptor.simpleCar(CollisionShape.box(1f, 0.4f, 2.2f), 1200f);
    }
}
