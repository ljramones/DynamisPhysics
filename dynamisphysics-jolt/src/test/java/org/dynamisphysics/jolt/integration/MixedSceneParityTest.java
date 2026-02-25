package org.dynamisphysics.jolt.integration;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.jolt.JoltBackendRegistrar;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.parity.integration", matches = "true")
class MixedSceneParityTest {

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void mixedSceneRemainsStableAndInteractive(PhysicsBackend backend) {
        PhysicsWorld world = IntegrationParityScenes.newWorld(backend, true);
        List<RigidBodyHandle> tracked = new ArrayList<>();
        try {
            IntegrationParityScenes.spawnGroundBox(world);

            VehicleHandle vehicle = IntegrationParityScenes.spawnAndSettleVehicle(world, -3f, 1.0f, 0f);
            CharacterHandle character = IntegrationParityScenes.spawnCharacter(world);
            RagdollHandle ragdoll = IntegrationParityScenes.spawnRagdoll(world, 5f);
            tracked.addAll(IntegrationParityScenes.spawnCompounds(world, 3, -1f));
            IntegrationParityScenes.ConstraintModule spring = IntegrationParityScenes.addSpringModule(world, -4f);
            IntegrationParityScenes.ConstraintModule gear = IntegrationParityScenes.addMechanicalModule(world, 3f);

            for (int i = 0; i < 3; i++) {
                tracked.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.3f), 1f)
                    .worldTransform(new Matrix4f().identity().translation(-2f + i, 4f + i * 0.2f, -1f))
                    .build()));
            }

            float startSpeed = world.getVehicleState(vehicle).speed();
            float startCharacterY = world.getCharacterState(character).position().y();
            float maxCharacterY = world.getCharacterState(character).position().y();
            int contactEvents = 0;
            int allEvents = 0;
            for (int i = 0; i < 300; i++) {
                if (i < 140) {
                    world.applyThrottle(vehicle, 0.4f);
                    world.moveCharacter(character, new Vector3f(1.2f, 0f, 0f));
                }
                if (i == 60) {
                    world.jumpCharacter(character, 3.5f);
                }
                world.step(IntegrationParityScenes.DT, 1);
                BodyState chassisState = world.getBodyState(vehicle.chassisBody());
                assertTrue(IntegrationParityScenes.stateFinite(chassisState), "non-finite chassis state at step " + i);

                for (PhysicsEvent event : world.drainEvents()) {
                    allEvents++;
                    if (event instanceof ContactEvent) {
                        contactEvents++;
                    }
                }
                maxCharacterY = java.lang.Math.max(maxCharacterY, world.getCharacterState(character).position().y());
            }

            float minY = IntegrationParityScenes.minY(world, tracked);
            float endSpeed = world.getVehicleState(vehicle).speed();
            float endCharacterY = world.getCharacterState(character).position().y();
            BodyState ragRoot = ragdoll.getBoneState("root");

            for (RigidBodyHandle handle : tracked) {
                assertTrue(IntegrationParityScenes.stateFinite(world.getBodyState(handle)),
                    "non-finite state for tracked body on " + backend);
            }
            assertTrue(minY > -5f, IntegrationParityScenes.summary(world, backend + "-minY", tracked, vehicle, character, allEvents));
            assertTrue(endSpeed > startSpeed + 0.5f,
                "vehicle did not accelerate for " + backend + ", start=" + startSpeed + " end=" + endSpeed);
            assertTrue(maxCharacterY > startCharacterY + 0.08f,
                "character jump not observed for " + backend + ", startY=" + startCharacterY + " maxY=" + maxCharacterY);
            assertTrue(endCharacterY < maxCharacterY, "character did not descend after jump for " + backend);
            assertTrue(IntegrationParityScenes.stateFinite(ragRoot), "ragdoll root non-finite for " + backend);
            assertTrue(contactEvents > 0, "no contact events observed for " + backend);
        } finally {
            world.destroy();
        }
    }
}
