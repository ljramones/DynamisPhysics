package org.dynamisengine.physics.api;

import org.dynamisengine.physics.api.body.BodyState;

public interface RagdollHandle {
    boolean isAlive();
    BodyState getBoneState(String animisBoneName);
}
