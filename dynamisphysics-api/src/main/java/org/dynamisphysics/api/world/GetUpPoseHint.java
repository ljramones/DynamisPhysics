package org.dynamisphysics.api.world;

import org.vectrix.core.Quaternionf;

public record GetUpPoseHint(
    String ragdollId,
    boolean faceDown,
    Quaternionf restOrientation
) {}
