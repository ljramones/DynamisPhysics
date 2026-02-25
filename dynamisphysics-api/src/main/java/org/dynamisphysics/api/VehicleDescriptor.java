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
) {}
