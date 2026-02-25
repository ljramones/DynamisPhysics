package org.dynamisphysics.ode4j.vehicle;

import org.dynamisphysics.api.TransmissionConfig;

final class Ode4jTransmissionModel {
    private static final float SHIFT_COOLDOWN_S = 0.3f;

    private final TransmissionConfig config;
    private int currentGear = 1;
    private float shiftCooldown = 0f;

    Ode4jTransmissionModel(TransmissionConfig config) {
        this.config = config;
    }

    float outputTorque(float engineTorque) {
        return engineTorque * currentGearRatio() * config.finalDriveRatio();
    }

    void update(float engineRpm, float dt) {
        shiftCooldown -= dt;
        if (!config.automatic() || shiftCooldown > 0f) {
            return;
        }
        if (engineRpm > config.shiftUpRpm() && currentGear < config.gearRatios().length) {
            currentGear++;
            shiftCooldown = SHIFT_COOLDOWN_S;
        } else if (engineRpm < config.shiftDownRpm() && currentGear > 1) {
            currentGear--;
            shiftCooldown = SHIFT_COOLDOWN_S;
        }
    }

    int currentGear() {
        return currentGear;
    }

    private float currentGearRatio() {
        if (currentGear == 0) {
            return config.reverseRatio();
        }
        int idx = org.vectrix.core.Math.clamp(currentGear - 1, 0, config.gearRatios().length - 1);
        return config.gearRatios()[idx];
    }
}
