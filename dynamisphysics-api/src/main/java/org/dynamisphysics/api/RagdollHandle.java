package org.dynamisphysics.api;

import org.dynamisphysics.api.body.BodyState;

public interface RagdollHandle {
    boolean isAlive();
    BodyState getBoneState(String animisBoneName);
}
