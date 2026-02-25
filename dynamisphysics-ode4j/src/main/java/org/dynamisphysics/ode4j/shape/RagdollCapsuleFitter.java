package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.meshforge.pack.buffer.PackedMesh;
import org.vectrix.core.Vector3f;

public final class RagdollCapsuleFitter {
    private static final float MIN_RADIUS = 0.02f;
    private static final float MIN_HEIGHT = 0.04f;

    private RagdollCapsuleFitter() {}

    public static CapsuleCollisionShape fitToBone(PackedMesh boneMesh, Vector3f boneAxis) {
        float[] positions = MeshForgeExtractors.extractPositions(boneMesh);
        ObbComputer.OrientedBoundingBox obb = ObbComputer.compute(positions);

        float height = obb.projectExtent(boneAxis);
        float radius = Math.max(obb.halfExtentA(), obb.halfExtentB()) * 0.5f;

        radius = Math.max(radius, MIN_RADIUS);
        height = Math.max(height, radius * 2f);
        height = Math.max(height, MIN_HEIGHT);

        return (CapsuleCollisionShape) CollisionShape.capsule(radius, height);
    }

    public static CapsuleCollisionShape fitToBoneYAxis(PackedMesh boneMesh) {
        return fitToBone(boneMesh, new Vector3f(0f, 1f, 0f));
    }
}
