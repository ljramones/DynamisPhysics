package org.dynamisphysics.api.config;

public record SolverConfig(
    int positionIterations,
    int velocityIterations,
    float erp,
    float cfm
) {
    public static SolverConfig defaults() {
        return new SolverConfig(10, 10, 0.2f, 1e-5f);
    }
}
