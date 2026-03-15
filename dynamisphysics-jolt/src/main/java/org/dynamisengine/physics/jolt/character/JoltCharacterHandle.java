package org.dynamisengine.physics.jolt.character;

import com.github.stephengold.joltjni.CharacterVirtual;
import org.dynamisengine.physics.api.CharacterDescriptor;
import org.dynamisengine.physics.api.CharacterHandle;
import org.dynamisengine.physics.api.material.PhysicsMaterial;
import org.dynamisengine.physics.api.world.CharacterState;
import org.dynamisengine.vectrix.core.Vector3f;

public final class JoltCharacterHandle implements CharacterHandle {
    private final CharacterDescriptor descriptor;
    private final CharacterVirtual character;

    private boolean alive = true;
    private CharacterState state;
    Vector3f intendedVelocity = new Vector3f();
    float pendingJumpImpulse = 0f;
    float verticalVelocity = 0f;
    boolean lastGrounded = false;

    public JoltCharacterHandle(CharacterDescriptor descriptor, CharacterVirtual character, Vector3f startPosition) {
        this.descriptor = descriptor;
        this.character = character;
        this.state = new CharacterState(
            new Vector3f(startPosition),
            new Vector3f(),
            false,
            PhysicsMaterial.DEFAULT,
            new Vector3f(0f, 1f, 0f),
            null,
            new Vector3f()
        );
    }

    public CharacterDescriptor descriptor() {
        return descriptor;
    }

    public CharacterVirtual character() {
        return character;
    }

    public void setState(CharacterState state) {
        this.state = state;
    }

    public void kill() {
        alive = false;
    }

    @Override
    public boolean isAlive() {
        return alive;
    }

    @Override
    public CharacterState getState() {
        return state;
    }
}
