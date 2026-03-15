package org.dynamisengine.physics.test.mock;

import org.dynamisengine.physics.api.VehicleHandle;
import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.world.WheelState;

public final class MockVehicleHandle implements VehicleHandle {
    private boolean alive = true;

    public void kill() {
        alive = false;
    }

    @Override public boolean isAlive() { return alive; }
    @Override public WheelState getWheelState(int i) { return WheelState.ZERO; }
    @Override public float getEngineRpm() { return 0f; }
    @Override public int getCurrentGear() { return 1; }
    @Override public float getSpeed() { return 0f; }
    @Override public RigidBodyHandle chassisBody() { return null; }
}
