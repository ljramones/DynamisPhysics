package org.dynamisengine.physics.api;

import org.dynamisengine.physics.api.world.CharacterState;

public interface CharacterHandle {
    boolean isAlive();
    CharacterState getState();
}
