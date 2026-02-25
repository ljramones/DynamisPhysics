package org.dynamisphysics.api.world;

public record PhysicsStats(
    float stepTimeMs,
    int bodyCount,
    int activeBodyCount,
    int sleepingBodyCount,
    int constraintCount,
    int islandCount,
    float broadPhaseMs,
    float narrowPhaseMs,
    float constraintSolveMs,
    float integrationMs
) {
    public static final PhysicsStats ZERO = new PhysicsStats(0f, 0, 0, 0, 0, 0, 0f, 0f, 0f, 0f);
}
