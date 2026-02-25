package org.dynamisphysics.api;

import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintType;

public record RagdollJointDesc(
    String parentBone,
    String childBone,
    ConstraintType jointType,
    ConstraintLimits limits
) {}
