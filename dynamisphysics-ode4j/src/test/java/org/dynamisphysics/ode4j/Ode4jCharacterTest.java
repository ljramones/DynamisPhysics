package org.dynamisphysics.ode4j;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.FootContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jCharacterTest {
    private PhysicsWorld world;

    @BeforeAll
    static void registerBackend() {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    @BeforeEach
    void setUp() {
        world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(100f, 0.1f, 100f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.GRASS)
            .worldTransform(new Matrix4f().translation(0f, -0.1f, 0f))
            .build());
    }

    @AfterEach
    void tearDown() {
        world.destroy();
    }

    @Test
    void characterSpawnReturnsLiveHandle() {
        CharacterHandle h = spawnDefaultCharacter();
        assertTrue(h.isAlive());
    }

    @Test
    void characterStateIsAvailableAfterSpawn() {
        CharacterHandle h = spawnDefaultCharacter();
        assertNotNull(world.getCharacterState(h));
    }

    @Test
    void characterMoveChangesHorizontalPosition() {
        CharacterHandle h = spawnDefaultCharacter();
        settle(10);
        float beforeX = world.getCharacterState(h).position().x();

        world.moveCharacter(h, new Vector3f(2f, 0f, 0f));
        settle(60);

        float afterX = world.getCharacterState(h).position().x();
        assertTrue(afterX > beforeX + 0.05f, "Character should move forward");
    }

    @Test
    void jumpAddsUpwardVelocity() {
        CharacterHandle h = spawnDefaultCharacter();
        settle(60);
        assertTrue(world.getCharacterState(h).isGrounded(), "Character should be grounded before jump");
        float beforeY = world.getCharacterState(h).position().y();

        world.jumpCharacter(h, 5f);
        world.step(1f / 60f);
        world.step(1f / 60f);
        world.step(1f / 60f);

        float afterY = world.getCharacterState(h).position().y();
        assertTrue(afterY > beforeY + 0.01f, "Jump should move character upward");
    }

    @Test
    void characterBecomesGroundedOnFlatGround() {
        CharacterHandle h = spawnDefaultCharacter();
        settle(60);
        assertTrue(world.getCharacterState(h).isGrounded(), "Character should be grounded on flat ground");
    }

    @Test
    void footContactEventFiresWhenCharacterGrounded() {
        CharacterHandle h = spawnDefaultCharacter();
        List<PhysicsEvent> events = new ArrayList<>();
        for (int i = 0; i < 60; i++) {
            world.step(1f / 60f);
            events.addAll(world.drainEvents());
        }
        assertEventFired(events, FootContactEvent.class);
    }

    @Test
    void footContactHintCallbackIsInvoked() {
        AtomicInteger hints = new AtomicInteger();
        CharacterHandle h = world.spawnCharacter(new CharacterDescriptor(
            1.8f,
            0.3f,
            80f,
            0.35f,
            45f,
            200f,
            0.02f,
            PhysicsMaterial.DEFAULT,
            1,
            -1,
            hint -> {
                if (hint != null && hint.grounded()) {
                    hints.incrementAndGet();
                }
            }
        ));
        assertTrue(h.isAlive());

        settle(60);
        assertTrue(hints.get() > 0, "Expected grounded FootContactHint callback");
    }

    @Test
    void movingPlatformVelocityIsInherited() {
        RigidBodyHandle platform = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(2f, 0.2f, 2f), 1f)
            .mode(BodyMode.KINEMATIC)
            .worldTransform(new Matrix4f().translation(0f, 0.25f, 0f))
            .build());

        CharacterHandle h = spawnDefaultCharacter();
        settle(20);

        float beforeX = world.getCharacterState(h).position().x();
        for (int i = 0; i < 60; i++) {
            world.setVelocity(platform, new Vector3f(1f, 0f, 0f), new Vector3f());
            world.step(1f / 60f);
            world.drainEvents();
        }
        float afterX = world.getCharacterState(h).position().x();
        assertTrue(afterX > beforeX + 0.05f, "Character should inherit platform movement");
    }

    @Test
    void steepSlopeCausesSliding() {
        PhysicsWorld slopeWorld = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        try {
            slopeWorld.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(5f, 0.1f, 5f), 0f)
                .mode(BodyMode.STATIC)
                .material(PhysicsMaterial.ROCK)
                .worldTransform(new Matrix4f().rotateX((float) java.lang.Math.toRadians(60)).translate(0f, 0.5f, 0f))
                .build());

            CharacterHandle h = slopeWorld.spawnCharacter(new CharacterDescriptor(
                1.8f,
                0.3f,
                80f,
                0.35f,
                45f,
                200f,
                0.02f,
                PhysicsMaterial.DEFAULT,
                1,
                -1
            ));
            for (int i = 0; i < 90; i++) {
                slopeWorld.step(1f / 60f);
                slopeWorld.drainEvents();
            }

            assertFalse(slopeWorld.getCharacterState(h).isGrounded(), "Steep slope should not be treated as grounded");
        } finally {
            slopeWorld.destroy();
        }
    }

    @Test
    void destroyCharacterKillsHandle() {
        CharacterHandle h = spawnDefaultCharacter();
        world.destroyCharacter(h);
        assertFalse(h.isAlive());
    }

    private CharacterHandle spawnDefaultCharacter() {
        return world.spawnCharacter(new CharacterDescriptor(
            1.8f,
            0.3f,
            80f,
            0.35f,
            45f,
            200f,
            0.02f,
            PhysicsMaterial.DEFAULT,
            1,
            -1
        ));
    }

    private void settle(int steps) {
        for (int i = 0; i < steps; i++) {
            world.step(1f / 60f);
            world.drainEvents();
        }
    }
}
