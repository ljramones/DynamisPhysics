package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.TriangleMeshCollisionShape;
import org.meshforge.pack.buffer.PackedMesh;

public final class BvhTriangleMeshBuilder {
    private BvhTriangleMeshBuilder() {}

    public static TriangleMeshCollisionShape fromPackedMesh(PackedMesh mesh) {
        float[] positions = MeshForgeExtractors.extractPositions(mesh);
        int[] indices = MeshForgeExtractors.extractIndices(mesh);
        if (indices.length < 3) {
            indices = MeshForgeExtractors.sequentialTriangles(positions);
        }
        return (TriangleMeshCollisionShape) CollisionShape.triangleMesh(positions, indices);
    }
}
