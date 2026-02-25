package org.dynamisphysics.api;

import org.vectrix.core.Vector3f;

public record AerodynamicsConfig(
    float dragCoefficient,
    float downforceCoefficient,
    float frontalArea,
    Vector3f centreOfPressure
) {}
