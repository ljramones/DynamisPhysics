package org.dynamisphysics.ode4j.debug;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;

public final class DebugColourPolicy {
    private DebugColourPolicy() {}

    public static float[] colourForBody(Ode4jBodyHandle body, boolean showSleepState) {
        if (body.config().isSensor()) {
            return rgba(1f, 0.85f, 0.2f, 1f);
        }
        if (body.mode() == BodyMode.KINEMATIC) {
            return rgba(0.2f, 0.45f, 1f, 1f);
        }
        if (body.mode() == BodyMode.STATIC) {
            return rgba(0.6f, 0.6f, 0.6f, 1f);
        }
        boolean sleeping = body.body() == null || !body.body().isEnabled();
        if (showSleepState && sleeping) {
            return rgba(0.45f, 0.45f, 0.45f, 1f);
        }
        return rgba(0.2f, 0.9f, 0.2f, 1f);
    }

    public static float[] contactColour() {
        return rgba(1f, 0.15f, 0.15f, 1f);
    }

    public static float[] constraintColour() {
        return rgba(1f, 0.6f, 0.1f, 1f);
    }

    public static float[] velocityColour() {
        return rgba(0.1f, 0.95f, 1f, 1f);
    }

    private static float[] rgba(float r, float g, float b, float a) {
        return new float[] { r, g, b, a };
    }
}
