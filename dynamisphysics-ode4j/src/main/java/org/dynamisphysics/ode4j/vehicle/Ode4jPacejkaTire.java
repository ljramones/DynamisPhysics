package org.dynamisphysics.ode4j.vehicle;

import org.dynamisphysics.api.PacejkaCoeffs;

public final class Ode4jPacejkaTire {
    private Ode4jPacejkaTire() {}

    public static float longitudinalForce(float slipRatio, PacejkaCoeffs c, float normalLoad) {
        float bx = c.stiffness() * slipRatio;
        float force = c.peak() * org.vectrix.core.Math.sin(
            c.shape() * org.vectrix.core.Math.atan(
                bx - c.curvature() * (bx - org.vectrix.core.Math.atan(bx))
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
        float ref = org.vectrix.core.Math.max(org.vectrix.core.Math.abs(chassisSpeedMs), 0.5f);
        return (wheelSpeed - chassisSpeedMs) / ref;
    }
}
