package org.dynamisphysics.ode4j.vehicle;

import org.dynamisphysics.api.PacejkaCoeffs;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.WheelConfig;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.event.VehicleAirborneEvent;
import org.dynamisphysics.api.event.WheelSlipEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.world.VehicleState;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.dynamisphysics.ode4j.event.Ode4jEventBuffer;
import org.dynamisphysics.ode4j.query.Ode4jRaycastExecutor;
import org.ode4j.math.DVector3;
import org.vectrix.core.Vector3f;
import org.vectrix.physics.SpringDamperf;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public final class Ode4jVehicleSystem {
    private static final float SLIP_EVENT_THRESHOLD = 0.15f;
    private static final float AIRBORNE_RAY_EXTENSION = 0.1f;

    private final Ode4jBodyRegistry bodyRegistry;
    private final Ode4jEventBuffer eventBuffer;
    private final Ode4jRaycastExecutor raycastExecutor;

    private final Map<VehicleHandle, Ode4jVehicleHandle> vehicles = new LinkedHashMap<>();
    private final Map<VehicleHandle, Ode4jEngineModel> engineModels = new LinkedHashMap<>();
    private final Map<VehicleHandle, Ode4jTransmissionModel> transmissionModels = new LinkedHashMap<>();

    public Ode4jVehicleSystem(
        Ode4jBodyRegistry bodyRegistry,
        Ode4jEventBuffer eventBuffer,
        Ode4jRaycastExecutor raycastExecutor
    ) {
        this.bodyRegistry = bodyRegistry;
        this.eventBuffer = eventBuffer;
        this.raycastExecutor = raycastExecutor;
    }

    public VehicleHandle spawn(VehicleDescriptor desc) {
        RigidBodyHandle chassis = bodyRegistry.spawn(
            RigidBodyConfig.builder(desc.chassisShape(), desc.chassisMass())
                .material(desc.chassisMaterial())
                .ccd(true)
                .build()
        );
        var handle = new Ode4jVehicleHandle(desc, chassis);
        vehicles.put(handle, handle);
        engineModels.put(handle, new Ode4jEngineModel(desc.engine()));
        transmissionModels.put(handle, new Ode4jTransmissionModel(desc.transmission()));
        return handle;
    }

    public void destroy(VehicleHandle h) {
        Ode4jVehicleHandle oh = vehicles.remove(h);
        if (oh == null) {
            return;
        }
        bodyRegistry.destroy(oh.chassisBody());
        engineModels.remove(h);
        transmissionModels.remove(h);
        oh.kill();
    }

    public void applyThrottle(VehicleHandle h, float v) { vh(h).throttle = org.vectrix.core.Math.clamp(v, 0f, 1f); }
    public void applyBrake(VehicleHandle h, float v) { vh(h).brake = org.vectrix.core.Math.clamp(v, 0f, 1f); }
    public void applySteering(VehicleHandle h, float v) { vh(h).steeringAngle = v; }
    public void applyHandbrake(VehicleHandle h, boolean e) { vh(h).handbrakeEngaged = e; }

    public VehicleState getVehicleState(VehicleHandle h) {
        Ode4jVehicleHandle oh = vh(h);
        BodyState chassisState = bodyRegistry.getState(oh.chassisBody());
        List<org.dynamisphysics.api.world.WheelState> wheels = oh.internalState().wheelStates
            .stream()
            .map(Ode4jWheelState::toPublic)
            .collect(Collectors.toList());
        return new VehicleState(
            chassisState,
            wheels,
            oh.getEngineRpm(),
            oh.internalState().engineTorque,
            oh.getCurrentGear(),
            oh.getSpeed()
        );
    }

    public int vehicleCount() {
        return vehicles.size();
    }

    public void clearAll() {
        var snapshot = List.copyOf(vehicles.keySet());
        for (VehicleHandle h : snapshot) {
            destroy(h);
        }
    }

    public void stepAll(float dt) {
        for (Ode4jVehicleHandle vehicle : vehicles.values()) {
            stepVehicle(vehicle, dt);
        }
    }

    private void stepVehicle(Ode4jVehicleHandle vehicle, float dt) {
        VehicleDescriptor desc = vehicle.descriptor();
        Ode4jEngineModel engine = engineModels.get(vehicle);
        Ode4jTransmissionModel trans = transmissionModels.get(vehicle);
        Ode4jBodyHandle chassisBody = bodyRegistry.getHandle(vehicle.chassisBody());
        if (chassisBody == null || chassisBody.body() == null) {
            return;
        }

        BodyState chassisState = bodyRegistry.getState(vehicle.chassisBody());
        float chassisSpeed = chassisState.linearVelocity().length();
        vehicle.internalState().speed = chassisSpeed;

        boolean allAirborne = true;
        float totalDriveWheelLoad = 0f;
        for (int i = 0; i < desc.wheels().size(); i++) {
            WheelConfig wheel = desc.wheels().get(i);
            Ode4jWheelState out = vehicle.internalState().wheelStates.get(i);

            Vector3f attachWorld = transformPoint(chassisState, wheel.attachmentPoint());
            float rayLength = wheel.suspensionTravel() + wheel.radius() + AIRBORNE_RAY_EXTENSION;
            Optional<RaycastResult> hit = raycastExecutor.raycastClosest(
                attachWorld,
                new Vector3f(0f, -1f, 0f),
                rayLength,
                -1
            );

            if (hit.isEmpty()) {
                out.contactPosition = new Vector3f();
                out.contactNormal = new Vector3f();
                out.suspensionLength = wheel.suspensionTravel();
                out.slipRatio = 0f;
                out.angularVelocity = 0f;
                out.surfaceMaterial = PhysicsMaterial.DEFAULT;
                out.grounded = false;
                vehicle.prevCompression()[i] = 0f;
                continue;
            }

            allAirborne = false;
            RaycastResult hr = hit.get();
            float rayDist = hr.fraction() * rayLength;
            float compression = (wheel.suspensionTravel() + wheel.radius()) - rayDist;
            compression = org.vectrix.core.Math.clamp(compression, 0f, wheel.suspensionTravel());
            float compressionRate = (compression - vehicle.prevCompression()[i]) / dt;
            vehicle.prevCompression()[i] = compression;

            float suspForce = SpringDamperf.suspensionForce(
                compression,
                wheel.suspensionTravel(),
                wheel.springStiffness(),
                compressionRate,
                wheel.damping()
            );
            Vector3f normal = hr.normal();
            chassisBody.body().addForceAtPos(
                new DVector3(normal.x() * suspForce, normal.y() * suspForce, normal.z() * suspForce),
                new DVector3(hr.position().x(), hr.position().y(), hr.position().z())
            );

            PhysicsMaterial surface = hr.material();
            float normalLoad = suspForce;
            float wheelAngVel = estimateWheelAngularVelocity(vehicle, i, chassisSpeed, wheel.radius());
            if (vehicle.handbrakeEngaged && i >= 2) {
                wheelAngVel = 0f;
            }
            float slipRatio = Ode4jPacejkaTire.computeSlipRatio(wheelAngVel, wheel.radius(), chassisSpeed);
            float effectiveSlip = slipRatio / org.vectrix.core.Math.max(surface.friction(), 0.1f);
            effectiveSlip = org.vectrix.core.Math.clamp(effectiveSlip, -3f, 3f);

            PacejkaCoeffs scaledLong = wheel.longitudinalCoeffs()
                .withPeak(wheel.longitudinalCoeffs().peak() * surface.friction());
            float fx = Ode4jPacejkaTire.longitudinalForce(effectiveSlip, scaledLong, normalLoad);
            Vector3f longDir = chassisForwardDir(chassisState);
            chassisBody.body().addForceAtPos(
                new DVector3(longDir.x() * fx, longDir.y() * fx, longDir.z() * fx),
                new DVector3(hr.position().x(), hr.position().y(), hr.position().z())
            );

            if (vehicle.brake > 0f || vehicle.handbrakeEngaged) {
                boolean rearWheel = i >= 2;
                float brakeForce = (vehicle.handbrakeEngaged && rearWheel)
                    ? normalLoad * 0.8f
                    : normalLoad * org.vectrix.core.Math.clamp(vehicle.brake, 0f, 1f) * 0.6f;
                Vector3f chassisVel = chassisState.linearVelocity();
                Vector3f brakeDir = normalizeOrZero(chassisVel).negate(new Vector3f());
                chassisBody.body().addForceAtPos(
                    new DVector3(brakeDir.x() * brakeForce, brakeDir.y() * brakeForce, brakeDir.z() * brakeForce),
                    new DVector3(hr.position().x(), hr.position().y(), hr.position().z())
                );
            }

            if (wheel.driven()) {
                totalDriveWheelLoad += normalLoad;
            }

            if (org.vectrix.core.Math.abs(effectiveSlip) > SLIP_EVENT_THRESHOLD) {
                eventBuffer.add(new WheelSlipEvent(vehicle, i, effectiveSlip, hr.position(), surface));
            }

            out.contactPosition = hr.position();
            out.contactNormal = hr.normal();
            out.suspensionLength = wheel.suspensionTravel() - compression;
            out.slipRatio = effectiveSlip;
            out.angularVelocity = wheelAngVel;
            out.surfaceMaterial = surface;
            out.grounded = true;
        }

        if (allAirborne != vehicle.wasAirborne()) {
            eventBuffer.add(new VehicleAirborneEvent(vehicle, allAirborne));
            vehicle.setWasAirborne(allAirborne);
        }

        float drivetrainLoad = totalDriveWheelLoad * 0.05f;
        float engineTorque = engine.update(vehicle.throttle, drivetrainLoad, dt);
        trans.update(engine.currentRpm(), dt);
        float wheelTorque = trans.outputTorque(engineTorque);

        if (vehicle.throttle > 0f && !allAirborne) {
            float drivenRadius = desc.wheels().stream()
                .filter(WheelConfig::driven)
                .findFirst()
                .map(WheelConfig::radius)
                .orElse(0.35f);
            float driveForce = wheelTorque / org.vectrix.core.Math.max(drivenRadius, 0.01f);
            Vector3f fwd = chassisForwardDir(chassisState);
            chassisBody.body().addForce(new DVector3(fwd.x() * driveForce, fwd.y() * driveForce, fwd.z() * driveForce));
        }

        if (org.vectrix.core.Math.abs(vehicle.steeringAngle) > 0.001f && !allAirborne) {
            float yawTorque = vehicle.steeringAngle * chassisSpeed * 800f;
            chassisBody.body().addTorque(new DVector3(0.0, yawTorque, 0.0));
        }

        vehicle.internalState().engineRpm = engine.currentRpm();
        vehicle.internalState().engineTorque = engineTorque;
        vehicle.internalState().currentGear = trans.currentGear();
    }

    private Ode4jVehicleHandle vh(VehicleHandle h) {
        Ode4jVehicleHandle handle = vehicles.get(h);
        if (handle == null) {
            throw new IllegalArgumentException("Unknown vehicle handle: " + h);
        }
        return handle;
    }

    private static Vector3f transformPoint(BodyState state, Vector3f localPoint) {
        Vector3f rotated = state.orientation().transform(localPoint, new Vector3f());
        return state.position().add(rotated, new Vector3f());
    }

    private static Vector3f chassisForwardDir(BodyState state) {
        return normalizeOrZero(state.orientation().transform(new Vector3f(0f, 0f, -1f), new Vector3f()));
    }

    private static float estimateWheelAngularVelocity(
        Ode4jVehicleHandle vehicle,
        int wheelIndex,
        float chassisSpeed,
        float radius
    ) {
        WheelConfig wheel = vehicle.descriptor().wheels().get(wheelIndex);
        float base = chassisSpeed / org.vectrix.core.Math.max(radius, 0.01f);
        if (wheel.driven() && vehicle.throttle > 0f) {
            base += vehicle.throttle * 5f;
        }
        return base;
    }

    private static Vector3f normalizeOrZero(Vector3f v) {
        float len = v.length();
        if (len < 1e-6f) {
            return new Vector3f();
        }
        return new Vector3f(v).div(len);
    }
}
