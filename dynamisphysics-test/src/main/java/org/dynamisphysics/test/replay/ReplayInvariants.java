package org.dynamisphysics.test.replay;

public record ReplayInvariants(
    float minY,
    float maxSpeed,
    boolean requireFinite,
    boolean requireBodyCountStable
) {
    public static ReplayInvariants defaults() {
        return new ReplayInvariants(
            Float.parseFloat(System.getProperty("physics.replay.invariant.minY", "-10")),
            Float.parseFloat(System.getProperty("physics.replay.invariant.maxSpeed", "1000")),
            Boolean.parseBoolean(System.getProperty("physics.replay.invariant.requireFinite", "true")),
            Boolean.parseBoolean(System.getProperty("physics.replay.invariant.requireBodyCountStable", "true"))
        );
    }
}
