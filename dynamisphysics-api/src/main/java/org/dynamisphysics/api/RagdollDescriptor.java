package org.dynamisphysics.api;

import java.util.List;

public record RagdollDescriptor(
    List<RagdollBoneDesc> bones,
    List<RagdollJointDesc> joints,
    float totalMass
) {}
