package org.dynamisphysics.api.body;

import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

public record BodyState(
    Vector3f position,
    Quaternionf orientation,
    Vector3f linearVelocity,
    Vector3f angularVelocity,
    boolean sleeping
) {
    public static final BodyState ZERO = new BodyState(
        new Vector3f(),
        new Quaternionf(),
        new Vector3f(),
        new Vector3f(),
        false
    );
}
