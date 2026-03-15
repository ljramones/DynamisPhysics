package org.dynamisengine.physics.api.world;

import org.dynamisengine.vectrix.core.Quaternionf;

public record GetUpPoseHint(
    String ragdollId,
    boolean faceDown,
    Quaternionf restOrientation
) {}
