package org.dynamisphysics.api;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.world.WheelState;

public interface VehicleHandle {
    boolean isAlive();
    WheelState getWheelState(int wheelIndex);
    float getEngineRpm();
    int getCurrentGear();
    float getSpeed();
    RigidBodyHandle chassisBody();
}
