package org.dynamisphysics.test.scene;

import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.test.mock.MockPhysicsWorld;
import org.dynamisphysics.test.mock.TestCollisionShapes;
import org.vectrix.core.Matrix4f;

public final class SceneFactory {
    private SceneFactory() {}

    public static MockPhysicsWorld empty() {
        return new MockPhysicsWorld();
    }

    public static MockPhysicsWorld flatPlaneWithSphere(float sphereRadius, float spawnHeight) {
        var world = new MockPhysicsWorld();
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.sphere(sphereRadius), 1f)
            .worldTransform(new Matrix4f().translation(0f, spawnHeight, 0f))
            .build());
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.box(100f, 0.1f, 100f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .build());
        return world;
    }

    public static MockPhysicsWorld stackedBoxes(int n, float boxSize) {
        var world = new MockPhysicsWorld();
        for (int i = 0; i < n; i++) {
            world.spawnRigidBody(RigidBodyConfig.builder(
                    TestCollisionShapes.box(boxSize, boxSize, boxSize), 1f)
                .worldTransform(new Matrix4f().translation(0f, boxSize * (i + 0.5f), 0f))
                .build());
        }
        return world;
    }

    public static MockPhysicsWorld rampAndSphere() {
        var world = new MockPhysicsWorld();
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.box(5f, 0.1f, 2f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ROCK)
            .worldTransform(new Matrix4f()
                .rotateX((float) Math.toRadians(20))
                .translate(0f, 2f, 0f))
            .build());
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.sphere(0.3f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 4f, -2f))
            .build());
        return world;
    }

    public static MockPhysicsWorld twoBodies(float separation) {
        var world = new MockPhysicsWorld();
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.box(0.5f, 0.5f, 0.5f), 1f)
            .worldTransform(new Matrix4f().translation(-separation / 2f, 5f, 0f))
            .build());
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.box(0.5f, 0.5f, 0.5f), 1f)
            .worldTransform(new Matrix4f().translation(separation / 2f, 5f, 0f))
            .build());
        return world;
    }

    public static MockPhysicsWorld groundWithCharacter() {
        var world = new MockPhysicsWorld();
        world.spawnRigidBody(RigidBodyConfig.builder(
                TestCollisionShapes.box(100f, 0.1f, 100f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.GRASS)
            .build());
        world.spawnCharacter(new CharacterDescriptor(
            1.8f, 0.3f, 80f, 0.35f, 45f, 200f, 0.02f,
            PhysicsMaterial.DEFAULT, 1, -1));
        return world;
    }
}
