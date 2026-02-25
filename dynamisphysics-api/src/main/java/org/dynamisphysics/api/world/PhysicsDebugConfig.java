package org.dynamisphysics.api.world;

public record PhysicsDebugConfig(
    boolean showShapes,
    boolean showContacts,
    boolean showConstraints,
    boolean showVelocities,
    boolean showRaycasts,
    boolean showIslands,
    boolean showAabbs,
    boolean showSleepState
) {
    public static PhysicsDebugConfig all() {
        return new PhysicsDebugConfig(true, true, true, true, true, true, true, true);
    }

    public static PhysicsDebugConfig shapesOnly() {
        return new PhysicsDebugConfig(true, false, false, false, false, false, false, true);
    }
}
