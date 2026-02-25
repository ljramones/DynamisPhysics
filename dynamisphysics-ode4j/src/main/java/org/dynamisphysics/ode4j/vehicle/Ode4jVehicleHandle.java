package org.dynamisphysics.ode4j.vehicle;

import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.world.WheelState;

public final class Ode4jVehicleHandle implements VehicleHandle {
    private final VehicleDescriptor descriptor;
    private final RigidBodyHandle chassis;
    private final Ode4jVehicleState state;
    private final float[] prevCompression;
    private boolean alive = true;
    private boolean wasAirborne = false;

    volatile float throttle = 0f;
    volatile float brake = 0f;
    volatile float steeringAngle = 0f;
    volatile boolean handbrakeEngaged = false;

    public Ode4jVehicleHandle(VehicleDescriptor descriptor, RigidBodyHandle chassis) {
        this.descriptor = descriptor;
        this.chassis = chassis;
        this.state = new Ode4jVehicleState(descriptor.wheels().size());
        this.prevCompression = new float[descriptor.wheels().size()];
    }

    public VehicleDescriptor descriptor() { return descriptor; }
    public float[] prevCompression() { return prevCompression; }
    public boolean wasAirborne() { return wasAirborne; }
    public void setWasAirborne(boolean value) { wasAirborne = value; }
    public Ode4jVehicleState internalState() { return state; }
    public void kill() { alive = false; }

    @Override public boolean isAlive() { return alive; }
    @Override public RigidBodyHandle chassisBody() { return chassis; }
    @Override public float getEngineRpm() { return state.engineRpm; }
    @Override public int getCurrentGear() { return state.currentGear; }
    @Override public float getSpeed() { return state.speed; }
    @Override public WheelState getWheelState(int wheelIndex) { return state.wheelStates.get(wheelIndex).toPublic(); }
}
