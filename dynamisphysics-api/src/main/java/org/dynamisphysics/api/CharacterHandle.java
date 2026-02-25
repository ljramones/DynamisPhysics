package org.dynamisphysics.api;

import org.dynamisphysics.api.world.CharacterState;

public interface CharacterHandle {
    boolean isAlive();
    CharacterState getState();
}
