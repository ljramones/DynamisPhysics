package org.dynamisphysics.api.event;

import org.vectrix.core.Vector3f;

public record ContactPoint(
    Vector3f position,
    Vector3f normal,
    float depth,
    float impulse
) {}
