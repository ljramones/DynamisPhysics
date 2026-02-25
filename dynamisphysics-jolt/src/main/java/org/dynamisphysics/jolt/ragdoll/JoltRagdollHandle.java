package org.dynamisphysics.jolt.ragdoll;

import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyHandle;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;

public final class JoltRagdollHandle implements RagdollHandle {
    private final String id;
    private final Map<String, RigidBodyHandle> bones;
    private final Function<String, BodyState> stateLookup;
    private boolean alive = true;

    public JoltRagdollHandle(String id, Map<String, RigidBodyHandle> bones, Function<String, BodyState> stateLookup) {
        this.id = id;
        this.bones = Collections.unmodifiableMap(new LinkedHashMap<>(bones));
        this.stateLookup = stateLookup;
    }

    public String id() {
        return id;
    }

    public Map<String, RigidBodyHandle> bones() {
        return bones;
    }

    public void kill() {
        alive = false;
    }

    @Override
    public boolean isAlive() {
        return alive;
    }

    @Override
    public BodyState getBoneState(String animisBoneName) {
        return stateLookup.apply(animisBoneName);
    }
}

