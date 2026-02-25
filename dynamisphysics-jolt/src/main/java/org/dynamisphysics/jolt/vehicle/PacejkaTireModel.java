package org.dynamisphysics.jolt.vehicle;

import org.dynamisphysics.api.PacejkaCoeffs;

final class PacejkaTireModel {
    private PacejkaTireModel() {
    }

    static float longitudinalForce(float slipRatio, PacejkaCoeffs c, float normalLoad) {
        float bx = c.stiffness() * slipRatio;
        float f = c.peak() * org.vectrix.core.Math.sin(
            c.shape() * org.vectrix.core.Math.atan(
                bx - c.curvature() * (bx - org.vectrix.core.Math.atan(bx))
            )
        );
        return f * normalLoad;
    }

    static float computeSlipRatio(float wheelAngularVel, float wheelRadius, float chassisSpeedMs) {
        float wheelSpeed = wheelAngularVel * wheelRadius;
        float ref = org.vectrix.core.Math.max(org.vectrix.core.Math.abs(chassisSpeedMs), 0.5f);
        return (wheelSpeed - chassisSpeedMs) / ref;
    }
}
