package org.dynamisengine.physics.test.mock;

import org.dynamisengine.physics.api.CharacterHandle;
import org.dynamisengine.physics.api.world.CharacterState;

public final class MockCharacterHandle implements CharacterHandle {
    private boolean alive = true;

    public void kill() {
        alive = false;
    }

    @Override public boolean isAlive() { return alive; }
    @Override public CharacterState getState() { return CharacterState.ZERO; }
}
