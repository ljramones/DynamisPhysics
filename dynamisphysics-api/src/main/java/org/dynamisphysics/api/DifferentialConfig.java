package org.dynamisphysics.api;

public record DifferentialConfig(
    DifferentialMode mode,
    float limitedSlipBias
) {}
