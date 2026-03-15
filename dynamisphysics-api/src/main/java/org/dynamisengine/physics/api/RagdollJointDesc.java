package org.dynamisengine.physics.api;

import org.dynamisengine.physics.api.constraint.ConstraintLimits;
import org.dynamisengine.physics.api.constraint.ConstraintType;

public record RagdollJointDesc(
    String parentBone,
    String childBone,
    ConstraintType jointType,
    ConstraintLimits limits
) {}
