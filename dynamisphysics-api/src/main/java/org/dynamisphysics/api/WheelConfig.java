package org.dynamisphysics.api;

import org.dynamisengine.vectrix.core.Vector3f;

public record WheelConfig(
    Vector3f attachmentPoint,
    float radius,
    float width,
    float suspensionTravel,
    float springStiffness,
    float damping,
    PacejkaCoeffs longitudinalCoeffs,
    PacejkaCoeffs lateralCoeffs,
    boolean driven,
    boolean steered
) {}
