package org.dynamisengine.physics.api;

public record DifferentialConfig(
    DifferentialMode mode,
    float limitedSlipBias
) {}
