package org.dynamisphysics.test.mock;

import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.body.BodyState;

public final class MockRagdollHandle implements RagdollHandle {
    private boolean alive = true;

    public void kill() {
        alive = false;
    }

    @Override public boolean isAlive() { return alive; }
    @Override public BodyState getBoneState(String boneName) { return BodyState.ZERO; }
}
