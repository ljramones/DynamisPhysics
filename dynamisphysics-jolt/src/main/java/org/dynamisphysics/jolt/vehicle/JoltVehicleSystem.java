package org.dynamisphysics.jolt.vehicle;

import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.VehicleCollisionTester;
import com.github.stephengold.joltjni.VehicleCollisionTesterRay;
import com.github.stephengold.joltjni.VehicleConstraint;
import com.github.stephengold.joltjni.VehicleConstraintSettings;
import com.github.stephengold.joltjni.VehicleStepListener;
import com.github.stephengold.joltjni.Wheel;
import com.github.stephengold.joltjni.WheeledVehicleController;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.WheelConfig;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.event.VehicleAirborneEvent;
import org.dynamisphysics.api.event.WheelSlipEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.VehicleState;
import org.dynamisphysics.api.world.WheelState;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.dynamisphysics.jolt.event.JoltEventBuffer;
import org.vectrix.core.Vector3f;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.dynamisphysics.jolt.world.JoltConversions.toRVec3;
import static org.dynamisphysics.jolt.world.JoltConversions.toVec3;
import static org.dynamisphysics.jolt.world.JoltConversions.toVector3f;

public final class JoltVehicleSystem {
    private static final float SLIP_EVENT_THRESHOLD = 0.15f;
    private static final float DRIVE_FORCE_SCALE = 18_000f;
    private static final float BRAKE_FORCE_SCALE = 22_000f;
    private static final float STEER_TORQUE_SCALE = 1_200f;

    private final PhysicsSystem physicsSystem;
    private final JoltBodyRegistry bodyRegistry;
    private final JoltEventBuffer eventBuffer;
    private final Map<VehicleHandle, JoltVehicleHandle> vehicles = new LinkedHashMap<>();

    public JoltVehicleSystem(
        PhysicsSystem physicsSystem,
        JoltBodyRegistry bodyRegistry,
        JoltEventBuffer eventBuffer
    ) {
        this.physicsSystem = physicsSystem;
        this.bodyRegistry = bodyRegistry;
        this.eventBuffer = eventBuffer;
    }

    public VehicleHandle spawn(VehicleDescriptor descriptor) {
        RigidBodyHandle chassis = bodyRegistry.spawn(
            RigidBodyConfig.builder(descriptor.chassisShape(), descriptor.chassisMass())
                .material(descriptor.chassisMaterial())
                .ccd(true)
                .build()
        );
        JoltBodyHandle chassisHandle = bodyRegistry.getByHandle(chassis);
        if (chassisHandle == null) {
            throw new IllegalStateException("Unable to resolve spawned chassis handle");
        }

        Body body = resolveBody(chassisHandle.joltBodyId());
        VehicleConstraintSettings settings = JoltVehicleAdapter.toConstraintSettings(descriptor);
        VehicleConstraint constraint = new VehicleConstraint(body, settings);
        VehicleCollisionTester tester = new VehicleCollisionTesterRay(normalizeLayer(chassisHandle.layer()));
        constraint.setVehicleCollisionTester(tester);
        VehicleStepListener stepListener = constraint.getStepListener();
        physicsSystem.addStepListener(stepListener);
        physicsSystem.addConstraint(constraint);

        if (!(constraint.getController() instanceof WheeledVehicleController controller)) {
            throw new IllegalStateException("Expected WheeledVehicleController from VehicleConstraint");
        }

        JoltVehicleHandle handle = new JoltVehicleHandle(
            descriptor, chassis, constraint, controller, tester, stepListener
        );
        vehicles.put(handle, handle);
        return handle;
    }

    public void destroy(VehicleHandle handle) {
        JoltVehicleHandle vh = vehicles.remove(handle);
        if (vh == null) {
            return;
        }
        physicsSystem.removeStepListener(vh.stepListener());
        physicsSystem.removeConstraint(vh.constraint());
        bodyRegistry.destroy(vh.chassisBody());
        vh.kill();
    }

    public void applyThrottle(VehicleHandle handle, float value) {
        vh(handle).throttle = org.vectrix.core.Math.clamp(value, 0f, 1f);
    }

    public void applyBrake(VehicleHandle handle, float value) {
        vh(handle).brake = org.vectrix.core.Math.clamp(value, 0f, 1f);
    }

    public void applySteering(VehicleHandle handle, float value) {
        vh(handle).steering = org.vectrix.core.Math.clamp(value, -1f, 1f);
    }

    public void applyHandbrake(VehicleHandle handle, boolean engaged) {
        vh(handle).handbrake = engaged;
    }

    public VehicleState getVehicleState(VehicleHandle handle) {
        JoltVehicleHandle vh = vh(handle);
        BodyState chassis = bodyRegistry.getState(vh.chassisBody());
        return new VehicleState(
            chassis,
            List.copyOf(vh.wheelStates()),
            vh.getEngineRpm(),
            vh.engineTorque,
            vh.getCurrentGear(),
            vh.getSpeed()
        );
    }

    public void clearAll() {
        List<VehicleHandle> snapshot = List.copyOf(vehicles.keySet());
        for (VehicleHandle handle : snapshot) {
            destroy(handle);
        }
    }

    public void stepAll(float dt) {
        for (JoltVehicleHandle vehicle : vehicles.values()) {
            stepVehicle(vehicle, dt);
        }
    }

    private void stepVehicle(JoltVehicleHandle vehicle, float dt) {
        JoltBodyHandle chassisHandle = bodyRegistry.getByHandle(vehicle.chassisBody());
        if (chassisHandle == null || !chassisHandle.isAlive()) {
            return;
        }
        if (vehicle.throttle > 0f || vehicle.brake > 0f || vehicle.handbrake || org.vectrix.core.Math.abs(vehicle.steering) > 0f) {
            physicsSystem.getBodyInterface().activateBody(chassisHandle.joltBodyId());
        }
        vehicle.controller().setDriverInput(
            vehicle.throttle,
            vehicle.steering,
            vehicle.brake,
            vehicle.handbrake ? 1f : 0f
        );

        BodyState chassisState = bodyRegistry.getState(vehicle.chassisBody());
        float speed = horizontalSpeed(chassisState.linearVelocity());
        float engineRpm = vehicle.controller().getEngine().getCurrentRpm();
        float engineTorque = vehicle.controller().getEngine().getTorque(engineRpm);
        int currentGear = vehicle.controller().getTransmission().getCurrentGear();
        vehicle.setTelemetry(speed, engineRpm, engineTorque, currentGear);

        boolean allAirborne = true;
        int groundedWheels = 0;
        float averageSurfaceFriction = 0f;
        for (int i = 0; i < vehicle.constraint().countWheels(); i++) {
            Wheel wheel = vehicle.constraint().getWheel(i);
            boolean grounded = wheel.hasContact();
            PhysicsMaterial surface = PhysicsMaterial.DEFAULT;
            Vector3f contactPosition = new Vector3f();
            Vector3f contactNormal = new Vector3f(0f, 1f, 0f);
            float suspensionLength = wheel.getSuspensionLength();
            float angularVelocity = wheel.getAngularVelocity();
            float slipRatio = 0f;

            if (grounded) {
                allAirborne = false;
                groundedWheels++;
                contactPosition = toVector3f(wheel.getContactPosition());
                contactNormal = toVector3f(wheel.getContactNormal());
                JoltBodyHandle contactBody = bodyRegistry.getByJoltId(wheel.getContactBodyId());
                if (contactBody != null) {
                    surface = contactBody.config().material();
                }
                averageSurfaceFriction += surface.friction();

                WheelConfig wheelConfig = vehicle.descriptor().wheels().get(i);
                float baseSlip = PacejkaTireModel.computeSlipRatio(angularVelocity, wheelConfig.radius(), speed);
                float tractionPenalty = vehicle.throttle * (1f - org.vectrix.core.Math.clamp(surface.friction(), 0f, 1f));
                slipRatio = org.vectrix.core.Math.clamp(baseSlip + tractionPenalty, -3f, 3f);

                if (org.vectrix.core.Math.abs(slipRatio) > SLIP_EVENT_THRESHOLD) {
                    eventBuffer.add(new WheelSlipEvent(vehicle, i, slipRatio, contactPosition, surface));
                }
            }

            vehicle.setWheelState(i, new WheelState(
                contactPosition,
                contactNormal,
                suspensionLength,
                slipRatio,
                angularVelocity,
                surface,
                grounded
            ));
        }

        if (groundedWheels > 0) {
            averageSurfaceFriction /= groundedWheels;
            applyDriveAssist(vehicle, chassisState, dt, averageSurfaceFriction);
        }

        if (allAirborne != vehicle.wasAirborne()) {
            eventBuffer.add(new VehicleAirborneEvent(vehicle, allAirborne));
            vehicle.setWasAirborne(allAirborne);
        }
    }

    public int vehicleCount() {
        return vehicles.size();
    }

    private JoltVehicleHandle vh(VehicleHandle handle) {
        JoltVehicleHandle vh = vehicles.get(handle);
        if (vh == null) {
            throw new IllegalArgumentException("Unknown vehicle handle: " + handle);
        }
        return vh;
    }

    private Body resolveBody(int bodyId) {
        BodyLockRead lock = new BodyLockRead(physicsSystem.getBodyLockInterface(), bodyId);
        try {
            if (!lock.succeeded()) {
                throw new IllegalStateException("Unable to lock body id " + bodyId);
            }
            var body = lock.getBody();
            if (!(body instanceof Body mutableBody)) {
                throw new IllegalStateException("Expected mutable Body for id " + bodyId);
            }
            return mutableBody;
        } finally {
            lock.releaseLock();
        }
    }

    private static int normalizeLayer(int layer) {
        return Math.floorMod(layer, 256);
    }

    private void applyDriveAssist(JoltVehicleHandle vehicle, BodyState chassisState, float dt, float surfaceFriction) {
        float traction = org.vectrix.core.Math.clamp(surfaceFriction, 0.05f, 1f);
        Vector3f forward = chassisForward(chassisState);
        Vector3f position = chassisState.position();
        Vector3f velocity = chassisState.linearVelocity();

        if (vehicle.throttle > 0f) {
            float driveForce = vehicle.throttle * DRIVE_FORCE_SCALE * traction;
            Vector3f force = new Vector3f(forward).mul(driveForce);
            bodyRegistry.applyForce(vehicle.chassisBody(), toVec3(force), toRVec3(position));
        }

        float horizontalSpeed = horizontalSpeed(velocity);
        if (vehicle.brake > 0f && horizontalSpeed > 0.001f) {
            Vector3f opposite = new Vector3f(velocity.x(), 0f, velocity.z()).normalize().negate();
            float brakeForce = vehicle.brake * BRAKE_FORCE_SCALE * traction;
            Vector3f force = opposite.mul(brakeForce);
            bodyRegistry.applyForce(vehicle.chassisBody(), toVec3(force), toRVec3(position));
        }

        if (org.vectrix.core.Math.abs(vehicle.steering) > 0f && horizontalSpeed > 0.001f) {
            float yawTorque = vehicle.steering * horizontalSpeed * STEER_TORQUE_SCALE * traction * dt;
            bodyRegistry.applyTorque(vehicle.chassisBody(), toVec3(new Vector3f(0f, yawTorque, 0f)));
        }
    }

    private static Vector3f chassisForward(BodyState state) {
        return state.orientation().transform(new Vector3f(0f, 0f, -1f), new Vector3f()).normalize();
    }

    private static float horizontalSpeed(Vector3f velocity) {
        return (float) java.lang.Math.sqrt(velocity.x() * velocity.x() + velocity.z() * velocity.z());
    }
}
