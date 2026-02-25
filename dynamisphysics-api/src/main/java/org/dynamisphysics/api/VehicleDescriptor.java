package org.dynamisphysics.api;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Vector3f;

import java.util.List;

public record VehicleDescriptor(
    CollisionShape chassisShape,
    float chassisMass,
    Vector3f centreOfMass,
    List<WheelConfig> wheels,
    EngineConfig engine,
    TransmissionConfig transmission,
    DifferentialConfig differential,
    AerodynamicsConfig aerodynamics,
    AntiRollConfig antiRoll,
    PhysicsMaterial chassisMaterial
) {
    public static VehicleDescriptor simpleCar(CollisionShape chassisShape, float chassisMass) {
        return new VehicleDescriptor(
            chassisShape,
            chassisMass,
            new Vector3f(0f, -0.2f, 0f),
            List.of(
                wheelAt(-0.75f, -0.4f, 1.4f, true, true),
                wheelAt(0.75f, -0.4f, 1.4f, true, true),
                wheelAt(-0.75f, -0.4f, -1.4f, true, false),
                wheelAt(0.75f, -0.4f, -1.4f, true, false)
            ),
            defaultEngine(),
            defaultTransmission(),
            new DifferentialConfig(DifferentialMode.OPEN, 0f),
            new AerodynamicsConfig(0.3f, 0f, 2.2f, new Vector3f(0f, 0.5f, 0f)),
            new AntiRollConfig(8_000f, 6_000f),
            PhysicsMaterial.DEFAULT
        );
    }

    private static WheelConfig wheelAt(float x, float y, float z, boolean driven, boolean steered) {
        return new WheelConfig(
            new Vector3f(x, y, z),
            0.35f,
            0.2f,
            0.2f,
            25_000f,
            2_000f,
            PacejkaCoeffs.defaultLongitudinal(),
            PacejkaCoeffs.defaultLateral(),
            driven,
            steered
        );
    }

    private static EngineConfig defaultEngine() {
        return new EngineConfig(
            250f,
            6500f,
            900f,
            30f,
            new float[] {900f, 2000f, 4000f, 6000f, 6500f},
            new float[] {150f, 220f, 250f, 230f, 180f}
        );
    }

    private static TransmissionConfig defaultTransmission() {
        return new TransmissionConfig(
            true,
            new float[] {3.5f, 2.1f, 1.4f, 1.0f, 0.8f},
            -3.5f,
            3.7f,
            5800f,
            1800f,
            0.9f
        );
    }
}
