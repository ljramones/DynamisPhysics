package org.dynamisengine.physics.jolt.vehicle;

import org.dynamisengine.physics.api.PacejkaCoeffs;

final class PacejkaTireModel {
    private PacejkaTireModel() {
    }

    static float longitudinalForce(float slipRatio, PacejkaCoeffs c, float normalLoad) {
        float bx = c.stiffness() * slipRatio;
        float f = c.peak() * org.dynamisengine.vectrix.core.Math.sin(
            c.shape() * org.dynamisengine.vectrix.core.Math.atan(
                bx - c.curvature() * (bx - org.dynamisengine.vectrix.core.Math.atan(bx))
            )
        );
        return f * normalLoad;
    }

    static float computeSlipRatio(float wheelAngularVel, float wheelRadius, float chassisSpeedMs) {
        float wheelSpeed = wheelAngularVel * wheelRadius;
        float ref = org.dynamisengine.vectrix.core.Math.max(org.dynamisengine.vectrix.core.Math.abs(chassisSpeedMs), 0.5f);
        return (wheelSpeed - chassisSpeedMs) / ref;
    }
}
