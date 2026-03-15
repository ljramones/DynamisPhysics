package org.dynamisengine.physics.api;

import org.dynamisengine.physics.api.world.GetUpPoseHint;

import java.util.List;
import java.util.function.Consumer;

public record RagdollDescriptor(
    List<RagdollBoneDesc> bones,
    List<RagdollJointDesc> joints,
    float totalMass,
    Consumer<GetUpPoseHint> getUpHintListener
) {
    public RagdollDescriptor(
        List<RagdollBoneDesc> bones,
        List<RagdollJointDesc> joints,
        float totalMass
    ) {
        this(bones, joints, totalMass, null);
    }
}
