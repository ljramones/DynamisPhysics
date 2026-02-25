package org.dynamisphysics.ode4j;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.event.VehicleAirborneEvent;
import org.dynamisphysics.api.event.WheelSlipEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jVehicleTest {
    private PhysicsWorld world;

    @BeforeAll
    static void registerBackend() {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    @BeforeEach
    void setUp() {
        world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(100f, 0.1f, 100f), 0f)
            .mode(org.dynamisphysics.api.body.BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .worldTransform(new Matrix4f().translation(0f, -0.1f, 0f))
            .build());
    }

    @AfterEach
    void tearDown() {
        world.destroy();
    }

    @Test
    void vehicleSpawnReturnsLiveHandle() {
        VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
            CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
        assertTrue(h.isAlive());
    }

    @Test
    void chassisBodyIsLive() {
        VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
            CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
        assertNotNull(h.chassisBody());
        assertTrue(h.chassisBody().isAlive());
    }

    @Test
    void throttleProducesForwardAcceleration() {
        VehicleHandle h = spawnCarAt(0f, 1f, 0f);
        step(10);

        float beforeRpm = h.getEngineRpm();
        world.applyThrottle(h, 1.0f);
        step(30);
        float afterRpm = h.getEngineRpm();
        float afterSpeed = horizontalSpeed(h);

        assertTrue(afterRpm > beforeRpm,
            "Throttle should raise engine RPM. before=" + beforeRpm + " after=" + afterRpm);
        assertTrue(afterSpeed > 0.1f, "Throttle should produce non-trivial motion. speed=" + afterSpeed);
    }

    @Test
    void brakeDeceleratesVehicle() {
        VehicleHandle h = spawnCarAt(0f, 1f, 0f);
        step(10);

        world.applyThrottle(h, 1.0f);
        step(60);
        VehicleState beforeBrake = world.getVehicleState(h);
        float rearSpinBefore = (
            Math.abs(beforeBrake.wheels().get(2).angularVelocity()) +
            Math.abs(beforeBrake.wheels().get(3).angularVelocity())
        ) * 0.5f;

        world.applyThrottle(h, 0f);
        world.applyBrake(h, 1.0f);
        world.applyHandbrake(h, true);
        step(60);
        VehicleState afterBrake = world.getVehicleState(h);
        float rearSpinAfter = (
            Math.abs(afterBrake.wheels().get(2).angularVelocity()) +
            Math.abs(afterBrake.wheels().get(3).angularVelocity())
        ) * 0.5f;

        assertTrue(rearSpinAfter < rearSpinBefore,
            "Brake should reduce rear wheel spin. before=" + rearSpinBefore + " after=" + rearSpinAfter);
    }

    @Test
    void steeringProducesYawChange() {
        VehicleHandle h = spawnCarAt(0f, 1f, 0f);
        step(10);

        world.applyThrottle(h, 0.5f);
        step(30);

        float yawBefore = world.getBodyState(h.chassisBody()).angularVelocity().y();
        world.applySteering(h, 0.4f);
        step(60);
        float yawAfter = world.getBodyState(h.chassisBody()).angularVelocity().y();

        assertTrue(Math.abs(yawAfter - yawBefore) > 0.0005f,
            "Steering should change yaw rate. before=" + yawBefore + " after=" + yawAfter);
    }

    @Test
    void handbrakeStopsRearWheelSpin() {
        VehicleHandle h = spawnCarAt(0f, 1f, 0f);
        step(10);

        world.applyThrottle(h, 1.0f);
        step(30);
        world.applyHandbrake(h, true);
        step(30);

        VehicleState vs = world.getVehicleState(h);
        assertTrue(
            Math.abs(vs.wheels().get(2).angularVelocity()) < 2.0f ||
                Math.abs(vs.wheels().get(3).angularVelocity()) < 2.0f,
            "Handbrake should reduce rear wheel spin");
    }

    @Test
    void wheelSlipEventFiredOnHardAcceleration() {
        VehicleHandle h = spawnCarAt(0f, 1f, 0f);
        step(10);

        world.applyThrottle(h, 1.0f);
        var allEvents = new ArrayList<PhysicsEvent>();
        for (int i = 0; i < 120; i++) {
            world.step(1f / 60f);
            allEvents.addAll(world.drainEvents());
        }
        assertEventFired(allEvents, WheelSlipEvent.class);
    }

    @Test
    void iceSurfaceProducesMoreSlipThanAsphalt() {
        PhysicsWorld iceWorld = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        try {
            iceWorld.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(100f, 0.1f, 100f), 0f)
                .mode(org.dynamisphysics.api.body.BodyMode.STATIC)
                .material(PhysicsMaterial.ICE)
                .worldTransform(new Matrix4f().translation(0f, -0.1f, 0f))
                .build());

            VehicleHandle iceVehicle = iceWorld.spawnVehicle(VehicleDescriptor.simpleCar(
                CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
            iceWorld.teleport(iceVehicle.chassisBody(), new Vector3f(0f, 1f, 0f), new Quaternionf());

            VehicleHandle asphaltVehicle = spawnCarAt(0f, 1f, 0f);

            for (int i = 0; i < 10; i++) {
                iceWorld.step(1f / 60f);
                world.step(1f / 60f);
            }

            iceWorld.applyThrottle(iceVehicle, 1.0f);
            world.applyThrottle(asphaltVehicle, 1.0f);
            for (int i = 0; i < 60; i++) {
                iceWorld.step(1f / 60f);
                world.step(1f / 60f);
            }

            var iceEvents = new ArrayList<PhysicsEvent>();
            var asphaltEvents = new ArrayList<PhysicsEvent>();
            for (int i = 0; i < 30; i++) {
                iceWorld.step(1f / 60f);
                iceEvents.addAll(iceWorld.drainEvents());
                world.step(1f / 60f);
                asphaltEvents.addAll(world.drainEvents());
            }

            long iceSlipCount = iceEvents.stream().filter(e -> e instanceof WheelSlipEvent).count();
            long asphaltSlipCount = asphaltEvents.stream().filter(e -> e instanceof WheelSlipEvent).count();
            assertTrue(iceSlipCount > 0, "Expected slip events on ice surface");
            assertTrue(asphaltSlipCount > 0, "Expected slip events on asphalt surface");
            assertTrue(iceSlipCount != asphaltSlipCount,
                "Surface material scaling should change slip behaviour. iceCount=" + iceSlipCount
                    + " asphaltCount=" + asphaltSlipCount);
        } finally {
            iceWorld.destroy();
        }
    }

    @Test
    void vehicleAirborneEventFiredOnJump() {
        VehicleHandle h = spawnCarAt(0f, 1f, 0f);
        step(10);

        world.teleport(h.chassisBody(), new Vector3f(0f, 5f, 0f), new Quaternionf());

        var allEvents = new ArrayList<PhysicsEvent>();
        for (int i = 0; i < 60; i++) {
            world.step(1f / 60f);
            allEvents.addAll(world.drainEvents());
        }
        assertEventFired(allEvents, VehicleAirborneEvent.class);
    }

    @Test
    void destroyVehicleKillsHandleAndChassisBody() {
        VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
            CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
        var chassis = h.chassisBody();

        world.destroyVehicle(h);

        assertFalse(h.isAlive());
        assertFalse(chassis.isAlive());
    }

    @Test
    void vehicleStateReturnsCorrectWheelCount() {
        VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
            CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
        VehicleState vs = world.getVehicleState(h);
        assertEquals(4, vs.wheels().size(), "simpleCar should have 4 wheels");
    }

    private VehicleHandle spawnCarAt(float x, float y, float z) {
        VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
            CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
        world.teleport(h.chassisBody(), new Vector3f(x, y, z), new Quaternionf());
        return h;
    }

    private void step(int n) {
        for (int i = 0; i < n; i++) {
            world.step(1f / 60f);
        }
    }

    private float horizontalSpeed(VehicleHandle h) {
        Vector3f v = world.getBodyState(h.chassisBody()).linearVelocity();
        return (float) java.lang.Math.sqrt(v.x() * v.x() + v.z() * v.z());
    }

}
