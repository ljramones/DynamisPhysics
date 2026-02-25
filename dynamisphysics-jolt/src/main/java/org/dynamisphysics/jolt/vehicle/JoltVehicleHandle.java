package org.dynamisphysics.jolt.vehicle;

import com.github.stephengold.joltjni.VehicleCollisionTester;
import com.github.stephengold.joltjni.VehicleConstraint;
import com.github.stephengold.joltjni.VehicleStepListener;
import com.github.stephengold.joltjni.WheeledVehicleController;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.world.WheelState;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

public final class JoltVehicleHandle implements VehicleHandle {
    private final VehicleDescriptor descriptor;
    private final RigidBodyHandle chassisBody;
    private final VehicleConstraint constraint;
    private final WheeledVehicleController controller;
    private final VehicleCollisionTester collisionTester;
    private final VehicleStepListener stepListener;
    private final List<WheelState> wheelStates;

    private boolean alive = true;
    private boolean wasAirborne;

    float throttle;
    float brake;
    float steering;
    boolean handbrake;
    float speed;
    float engineRpm;
    float engineTorque;
    int currentGear;

    public JoltVehicleHandle(
        VehicleDescriptor descriptor,
        RigidBodyHandle chassisBody,
        VehicleConstraint constraint,
        WheeledVehicleController controller,
        VehicleCollisionTester collisionTester,
        VehicleStepListener stepListener
    ) {
        this.descriptor = descriptor;
        this.chassisBody = chassisBody;
        this.constraint = constraint;
        this.controller = controller;
        this.collisionTester = collisionTester;
        this.stepListener = stepListener;
        this.wheelStates = new ArrayList<>(descriptor.wheels().size());
        for (int i = 0; i < descriptor.wheels().size(); i++) {
            wheelStates.add(WheelState.ZERO);
        }
    }

    public VehicleDescriptor descriptor() {
        return descriptor;
    }

    public VehicleConstraint constraint() {
        return constraint;
    }

    public WheeledVehicleController controller() {
        return controller;
    }

    public VehicleCollisionTester collisionTester() {
        return collisionTester;
    }

    public VehicleStepListener stepListener() {
        return stepListener;
    }

    public List<WheelState> wheelStates() {
        return wheelStates;
    }

    public boolean wasAirborne() {
        return wasAirborne;
    }

    public void setWasAirborne(boolean airborne) {
        wasAirborne = airborne;
    }

    public void kill() {
        alive = false;
    }

    @Override
    public boolean isAlive() {
        return alive;
    }

    @Override
    public WheelState getWheelState(int wheelIndex) {
        return wheelStates.get(wheelIndex);
    }

    @Override
    public float getEngineRpm() {
        return engineRpm;
    }

    @Override
    public int getCurrentGear() {
        return currentGear;
    }

    @Override
    public float getSpeed() {
        return speed;
    }

    @Override
    public RigidBodyHandle chassisBody() {
        return chassisBody;
    }

    void setWheelState(int index, WheelState state) {
        wheelStates.set(index, state);
    }

    void setTelemetry(float speed, float engineRpm, float engineTorque, int currentGear) {
        this.speed = speed;
        this.engineRpm = engineRpm;
        this.engineTorque = engineTorque;
        this.currentGear = currentGear;
    }

    static Vector3f normalizeOrZero(Vector3f v) {
        float len = v.length();
        if (len < 1e-6f) {
            return new Vector3f();
        }
        return new Vector3f(v).div(len);
    }
}
