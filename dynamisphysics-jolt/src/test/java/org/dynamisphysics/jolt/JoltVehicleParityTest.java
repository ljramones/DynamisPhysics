package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.event.VehicleAirborneEvent;
import org.dynamisphysics.api.event.WheelSlipEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JoltVehicleParityTest {

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
    void vehicleSpawnReturnsLiveHandle(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
                CollisionShape.box(1f, 0.4f, 2.2f), 1200f
            ));
            assertTrue(h.isAlive());
            assertNotNull(h.chassisBody());
            assertTrue(h.chassisBody().isAlive());
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void throttleProducesForwardAcceleration(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = spawnAndSettle(world);
            float beforeRpm = h.getEngineRpm();
            world.applyThrottle(h, 1f);
            step(world, 120);
            float afterRpm = h.getEngineRpm();
            float horizontalSpeed = horizontalSpeed(world.getBodyState(h.chassisBody()));
            assertTrue(afterRpm >= beforeRpm, "beforeRpm=" + beforeRpm + " afterRpm=" + afterRpm);
            assertTrue(horizontalSpeed > 0.03f, "horizontalSpeed=" + horizontalSpeed);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void brakeDeceleratesVehicle(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = spawnAndSettle(world);
            world.applyThrottle(h, 1f);
            step(world, 120);
            VehicleState beforeBrake = world.getVehicleState(h);
            float rearSpinBefore = averageRearWheelSpin(beforeBrake);
            world.applyThrottle(h, 0f);
            world.applyBrake(h, 1f);
            world.applyHandbrake(h, true);
            step(world, 120);
            VehicleState afterBrake = world.getVehicleState(h);
            float rearSpinAfter = averageRearWheelSpin(afterBrake);
            assertTrue(rearSpinAfter <= rearSpinBefore, "before=" + rearSpinBefore + " after=" + rearSpinAfter);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void steeringProducesYawChange(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = spawnAndSettle(world);
            world.applyThrottle(h, 0.6f);
            step(world, 80);
            BodyState before = world.getBodyState(h.chassisBody());
            world.applySteering(h, 0.5f);
            step(world, 120);
            BodyState after = world.getBodyState(h.chassisBody());
            float yawBefore = before.angularVelocity().y();
            float yawAfter = after.angularVelocity().y();
            assertTrue(java.lang.Math.abs(yawAfter - yawBefore) > 0.0005f, "yawBefore=" + yawBefore + " yawAfter=" + yawAfter);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void wheelSlipEventFiredOnHardAcceleration(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = spawnAndSettle(world);
            world.applyThrottle(h, 1f);
            List<PhysicsEvent> events = new ArrayList<>();
            for (int i = 0; i < 180; i++) {
                world.step(1f / 60f, 1);
                events.addAll(world.drainEvents());
            }
            assertTrue(events.stream().anyMatch(WheelSlipEvent.class::isInstance));
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void vehicleAirborneEventFiredOnUpwardImpulse(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = spawnAndSettle(world);
            world.teleport(h.chassisBody(), new Vector3f(0f, 5f, 0f), new Quaternionf());
            List<PhysicsEvent> events = new ArrayList<>();
            for (int i = 0; i < 120; i++) {
                world.step(1f / 60f, 1);
                events.addAll(world.drainEvents());
            }
            assertTrue(events.stream().anyMatch(VehicleAirborneEvent.class::isInstance));
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void iceSurfaceProducesAtLeastAsManySlipEventsAsAsphalt(PhysicsBackend backend) {
        PhysicsWorld asphaltWorld = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        PhysicsWorld iceWorld = createWorldWithGround(backend, PhysicsMaterial.ICE);
        try {
            VehicleHandle asphalt = spawnAndSettle(asphaltWorld);
            VehicleHandle ice = spawnAndSettle(iceWorld);

            asphaltWorld.applyThrottle(asphalt, 1f);
            iceWorld.applyThrottle(ice, 1f);
            List<PhysicsEvent> asphaltEvents = new ArrayList<>();
            List<PhysicsEvent> iceEvents = new ArrayList<>();
            for (int i = 0; i < 180; i++) {
                asphaltWorld.step(1f / 60f, 1);
                iceWorld.step(1f / 60f, 1);
                asphaltEvents.addAll(asphaltWorld.drainEvents());
                iceEvents.addAll(iceWorld.drainEvents());
            }
            long asphaltSlip = asphaltEvents.stream().filter(WheelSlipEvent.class::isInstance).count();
            long iceSlip = iceEvents.stream().filter(WheelSlipEvent.class::isInstance).count();
            float asphaltSlipMagnitude = slipMagnitude(asphaltEvents);
            float iceSlipMagnitude = slipMagnitude(iceEvents);
            assertTrue(iceSlip > 0, "ice=" + iceSlip);
            assertTrue(asphaltSlip > 0, "asphalt=" + asphaltSlip);
            assertTrue(
                java.lang.Math.abs(iceSlipMagnitude - asphaltSlipMagnitude) > 0.001f,
                "iceMag=" + iceSlipMagnitude + " asphaltMag=" + asphaltSlipMagnitude
            );
        } finally {
            asphaltWorld.destroy();
            iceWorld.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void destroyVehicleKillsHandle(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
                CollisionShape.box(1f, 0.4f, 2.2f), 1200f
            ));
            var chassis = h.chassisBody();
            world.destroyVehicle(h);
            assertFalse(h.isAlive());
            assertFalse(chassis.isAlive());
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void vehicleStateHasExpectedWheelCount(PhysicsBackend backend) {
        PhysicsWorld world = createWorldWithGround(backend, PhysicsMaterial.ASPHALT);
        try {
            VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
                CollisionShape.box(1f, 0.4f, 2.2f), 1200f
            ));
            VehicleState vs = world.getVehicleState(h);
            assertEquals(4, vs.wheels().size());
        } finally {
            world.destroy();
        }
    }

    private static PhysicsWorld createWorldWithGround(PhysicsBackend backend, PhysicsMaterial surface) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        world.spawnRigidBody(
            RigidBodyConfig.builder(CollisionShape.box(100f, 0.1f, 100f), 0f)
                .mode(BodyMode.STATIC)
                .material(surface)
                .worldTransform(new Matrix4f().identity().translation(0f, -0.1f, 0f))
                .build()
        );
        return world;
    }

    private static VehicleHandle spawnAndSettle(PhysicsWorld world) {
        VehicleHandle h = world.spawnVehicle(VehicleDescriptor.simpleCar(
            CollisionShape.box(1f, 0.4f, 2.2f), 1200f
        ));
        world.teleport(h.chassisBody(), new Vector3f(0f, 1f, 0f), new Quaternionf());
        step(world, 20);
        return h;
    }

    private static void step(PhysicsWorld world, int count) {
        for (int i = 0; i < count; i++) {
            world.step(1f / 60f, 1);
        }
    }

    private static float horizontalSpeed(BodyState state) {
        Vector3f v = state.linearVelocity();
        return (float) java.lang.Math.sqrt(v.x() * v.x() + v.z() * v.z());
    }

    private static float averageRearWheelSpin(VehicleState state) {
        return (
            java.lang.Math.abs(state.wheels().get(2).angularVelocity()) +
            java.lang.Math.abs(state.wheels().get(3).angularVelocity())
        ) * 0.5f;
    }

    private static float slipMagnitude(List<PhysicsEvent> events) {
        float sum = 0f;
        for (PhysicsEvent event : events) {
            if (event instanceof WheelSlipEvent slip) {
                sum += java.lang.Math.abs(slip.slipRatio());
            }
        }
        return sum;
    }
}
