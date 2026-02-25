package org.dynamisphysics.ode4j.character;

import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.CharacterState;
import org.vectrix.core.Vector3f;

public final class Ode4jCharacterHandle implements CharacterHandle {
    private final CharacterDescriptor descriptor;

    private boolean alive = true;
    private CharacterState state;

    Vector3f intendedVelocity = new Vector3f();
    float jumpImpulse = 0f;
    float jumpGroundSnapCooldown = 0f;

    public Ode4jCharacterHandle(CharacterDescriptor descriptor, Vector3f startPosition) {
        this.descriptor = descriptor;
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

    public void setState(CharacterState state) {
        this.state = state;
    }

    @Override
    public boolean isAlive() {
        return alive;
    }

    public void kill() {
        alive = false;
    }

    @Override
    public CharacterState getState() {
        return state;
    }

    public RigidBodyHandle groundBody() {
        return state.groundBody();
    }
}
