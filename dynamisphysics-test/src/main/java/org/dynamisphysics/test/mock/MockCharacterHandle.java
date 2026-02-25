package org.dynamisphysics.test.mock;

import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.world.CharacterState;

public final class MockCharacterHandle implements CharacterHandle {
    private boolean alive = true;

    public void kill() {
        alive = false;
    }

    @Override public boolean isAlive() { return alive; }
    @Override public CharacterState getState() { return CharacterState.ZERO; }
}
