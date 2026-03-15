package org.dynamisengine.physics.api;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.world.WheelState;

public interface VehicleHandle {
    boolean isAlive();
    WheelState getWheelState(int wheelIndex);
    float getEngineRpm();
    int getCurrentGear();
    float getSpeed();
    RigidBodyHandle chassisBody();
}
