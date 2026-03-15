package org.dynamisengine.physics.api;

public record EngineConfig(
    float maxTorqueNm,
    float maxRpm,
    float idleRpm,
    float engineBrakeTorque,
    float[] torqueCurveRpm,
    float[] torqueCurveNm
) {}
