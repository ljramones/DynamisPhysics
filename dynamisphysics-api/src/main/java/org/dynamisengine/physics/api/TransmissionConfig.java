package org.dynamisengine.physics.api;

public record TransmissionConfig(
    boolean automatic,
    float[] gearRatios,
    float reverseRatio,
    float finalDriveRatio,
    float shiftUpRpm,
    float shiftDownRpm,
    float clutchStrength
) {}
