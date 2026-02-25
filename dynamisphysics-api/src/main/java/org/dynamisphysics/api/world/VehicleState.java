package org.dynamisphysics.api.world;

import org.dynamisphysics.api.body.BodyState;

import java.util.List;

public record VehicleState(
    BodyState chassis,
    List<WheelState> wheels,
    float engineRpm,
    float engineTorque,
    int currentGear,
    float speed
) {
    public static final VehicleState ZERO = new VehicleState(BodyState.ZERO, List.of(), 0f, 0f, 0, 0f);
}
