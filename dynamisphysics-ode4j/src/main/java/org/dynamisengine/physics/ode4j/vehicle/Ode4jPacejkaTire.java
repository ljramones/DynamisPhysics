package org.dynamisengine.physics.ode4j.vehicle;

import org.dynamisengine.physics.api.PacejkaCoeffs;

public final class Ode4jPacejkaTire {
    private Ode4jPacejkaTire() {}

    public static float longitudinalForce(float slipRatio, PacejkaCoeffs c, float normalLoad) {
        float bx = c.stiffness() * slipRatio;
        float force = c.peak() * org.dynamisengine.vectrix.core.Math.sin(
            c.shape() * org.dynamisengine.vectrix.core.Math.atan(
                bx - c.curvature() * (bx - org.dynamisengine.vectrix.core.Math.atan(bx))
            )
        );
        return force * normalLoad;
    }

    public static float lateralForce(float slipAngleRad, PacejkaCoeffs c, float normalLoad) {
        float slipTan = (float) java.lang.Math.tan(slipAngleRad);
        return longitudinalForce(slipTan, c, normalLoad);
    }

    public static float computeSlipRatio(float wheelAngularVel, float wheelRadius, float chassisSpeedMs) {
        float wheelSpeed = wheelAngularVel * wheelRadius;
        float ref = org.dynamisengine.vectrix.core.Math.max(org.dynamisengine.vectrix.core.Math.abs(chassisSpeedMs), 0.5f);
        return (wheelSpeed - chassisSpeedMs) / ref;
    }
}
