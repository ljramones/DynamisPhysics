package org.dynamisphysics.ode4j.vehicle;

import org.dynamisphysics.api.EngineConfig;

final class Ode4jEngineModel {
    private final EngineConfig config;
    private float currentRpm;

    Ode4jEngineModel(EngineConfig config) {
        this.config = config;
        this.currentRpm = config.idleRpm();
    }

    float update(float throttle, float drivetrainLoadTorque, float dt) {
        float clampedThrottle = org.vectrix.core.Math.clamp(throttle, 0f, 1f);
        float maxTorque = sampleTorqueCurve(currentRpm);
        float driverTorque = maxTorque * clampedThrottle;
        float netTorque = driverTorque - drivetrainLoadTorque - config.engineBrakeTorque() * (1f - clampedThrottle);
        float inertia = 0.15f;
        float alpha = netTorque / inertia;
        currentRpm += alpha * dt * (60f / (2f * org.vectrix.core.Math.PI_f));
        currentRpm = org.vectrix.core.Math.clamp(currentRpm, config.idleRpm(), config.maxRpm());
        return driverTorque;
    }

    float currentRpm() {
        return currentRpm;
    }

    private float sampleTorqueCurve(float rpm) {
        float[] rpmPoints = config.torqueCurveRpm();
        float[] torquePoints = config.torqueCurveNm();
        if (rpmPoints == null || torquePoints == null || rpmPoints.length == 0 || torquePoints.length == 0) {
            return config.maxTorqueNm();
        }
        for (int i = 0; i < rpmPoints.length - 1; i++) {
            if (rpm >= rpmPoints[i] && rpm <= rpmPoints[i + 1]) {
                float t = (rpm - rpmPoints[i]) / (rpmPoints[i + 1] - rpmPoints[i]);
                return torquePoints[i] + t * (torquePoints[i + 1] - torquePoints[i]);
            }
        }
        return rpm < rpmPoints[0] ? torquePoints[0] : torquePoints[torquePoints.length - 1];
    }
}
