package org.dynamisphysics.jolt;

import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.FootContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import static org.dynamiscollision.shapes.CollisionShape.box;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JoltCharacterParityTest {

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    static Stream<PhysicsBackend> backends() {
        return Stream.of(PhysicsBackend.ODE4J, PhysicsBackend.JOLT);
    }

    @ParameterizedTest
    @MethodSource("backends")
    void characterFallsAndLandsOnFlatPlane(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            CharacterHandle handle = world.spawnCharacter(defaultDescriptor());
            for (int i = 0; i < 180; i++) {
                world.step(1f / 60f, 1);
            }
            assertTrue(handle.isAlive());
            var state = world.getCharacterState(handle);
            assertTrue(state.isGrounded(),
                "expected grounded, got pos=" + state.position() + " vel=" + state.velocity());
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void moveCharacterTranslatesHorizontally(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            CharacterHandle handle = world.spawnCharacter(defaultDescriptor());
            for (int i = 0; i < 90; i++) {
                world.step(1f / 60f, 1);
            }
            float beforeX = world.getCharacterState(handle).position().x();
            world.moveCharacter(handle, new Vector3f(2f, 0f, 0f));
            for (int i = 0; i < 60; i++) {
                world.step(1f / 60f, 1);
            }
            float afterX = world.getCharacterState(handle).position().x();
            assertTrue(afterX > beforeX + 0.25f);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void jumpCharacterAddsUpwardVelocity(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            CharacterHandle handle = world.spawnCharacter(defaultDescriptor());
            for (int i = 0; i < 90; i++) {
                world.step(1f / 60f, 1);
            }
            world.jumpCharacter(handle, 4f);
            for (int i = 0; i < 6; i++) {
                world.step(1f / 60f, 1);
            }
            var state = world.getCharacterState(handle);
            assertTrue(state.velocity().y() > 0f,
                "expected positive Y velocity after jump, got " + state.velocity());
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void footContactEventFiredOnLanding(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            world.spawnCharacter(defaultDescriptor());
            List<PhysicsEvent> events = new ArrayList<>();
            for (int i = 0; i < 240; i++) {
                world.step(1f / 60f, 1);
                events.addAll(world.drainEvents());
            }
            assertTrue(events.stream().anyMatch(FootContactEvent.class::isInstance),
                "no FootContactEvent observed, event count=" + events.size());
        } finally {
            world.destroy();
        }
    }

    private static CharacterDescriptor defaultDescriptor() {
        return new CharacterDescriptor(
            1.8f,
            0.35f,
            80f,
            0.35f,
            45f,
            1000f,
            0.05f,
            PhysicsMaterial.DEFAULT,
            1,
            -1
        );
    }

    private static void spawnGround(PhysicsWorld world) {
        world.spawnRigidBody(
            RigidBodyConfig.builder(box(20f, 0.5f, 20f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().identity().translation(0f, -0.5f, 0f))
                .build()
        );
    }
}
