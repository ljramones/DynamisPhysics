package org.dynamisphysics.ode4j.vehicle;

import java.util.ArrayList;
import java.util.List;

final class Ode4jVehicleState {
    float engineRpm = 0f;
    float engineTorque = 0f;
    int currentGear = 1;
    float speed = 0f;
    final List<Ode4jWheelState> wheelStates;

    Ode4jVehicleState(int wheelCount) {
        wheelStates = new ArrayList<>(wheelCount);
        for (int i = 0; i < wheelCount; i++) {
            wheelStates.add(new Ode4jWheelState());
        }
    }
}
