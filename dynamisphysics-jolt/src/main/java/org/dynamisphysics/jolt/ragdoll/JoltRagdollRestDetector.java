package org.dynamisphysics.jolt.ragdoll;

import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;

final class JoltRagdollRestDetector {
    private static final float REST_LINEAR_SPEED = 0.05f;
    private static final float REST_ANGULAR_SPEED = 0.05f;
    private static final int REST_FRAMES_REQUIRED = 12;

    private int restFrames;

    boolean update(JoltRagdollSystem.Instance instance, JoltBodyRegistry bodyRegistry) {
        boolean allRest = true;
        for (RagdollBoneDesc bone : instance.descriptor().bones()) {
            RigidBodyHandle body = instance.handle().bones().get(bone.animisBoneName());
            if (body == null) {
                continue;
            }
            BodyState state = bodyRegistry.getState(body);
            if (state.linearVelocity().length() > REST_LINEAR_SPEED
                || state.angularVelocity().length() > REST_ANGULAR_SPEED) {
                allRest = false;
                break;
            }
        }
        if (allRest) {
            restFrames++;
        } else {
            restFrames = 0;
        }
        return restFrames >= REST_FRAMES_REQUIRED;
    }

    void reset() {
        restFrames = 0;
    }
}

