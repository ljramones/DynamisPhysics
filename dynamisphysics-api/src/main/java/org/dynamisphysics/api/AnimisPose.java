package org.dynamisphysics.api;

import org.vectrix.core.Matrix4f;

import java.util.Map;

/**
 * Lightweight pose carrier used by physics-facing ragdoll APIs.
 */
public record AnimisPose(Map<String, Matrix4f> boneWorldTransforms) {
    public static final AnimisPose EMPTY = new AnimisPose(Map.of());

    public Matrix4f boneWorldTransform(String boneName) {
        Matrix4f matrix = boneWorldTransforms.get(boneName);
        return matrix != null ? new Matrix4f(matrix) : new Matrix4f().identity();
    }
}
