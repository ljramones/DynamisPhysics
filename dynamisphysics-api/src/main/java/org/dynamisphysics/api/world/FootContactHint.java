package org.dynamisphysics.api.world;

import org.vectrix.core.Vector3f;

public record FootContactHint(
    String boneName,
    boolean grounded,
    Vector3f position,
    Vector3f normal,
    String materialTag
) {
    public static FootContactHint grounded(String bone, Vector3f pos, Vector3f normal, String tag) {
        return new FootContactHint(bone, true, pos, normal, tag);
    }

    public static FootContactHint airborne(String bone) {
        return new FootContactHint(bone, false, null, null, "");
    }
}
