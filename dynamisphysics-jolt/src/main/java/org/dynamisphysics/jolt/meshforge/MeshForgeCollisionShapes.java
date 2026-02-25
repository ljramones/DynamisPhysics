package org.dynamisphysics.jolt.meshforge;

import org.dynamiscollision.shapes.CollisionShape;
import org.meshforge.pack.buffer.PackedMesh;

public final class MeshForgeCollisionShapes {
    private MeshForgeCollisionShapes() {
    }

    public static CollisionShape triangleMesh(PackedMesh mesh) {
        return CollisionShape.triangleMesh(
            MeshForgeExtractors.extractPositions(mesh),
            MeshForgeExtractors.extractIndices(mesh)
        );
    }

    public static CollisionShape convexHull(PackedMesh mesh) {
        return CollisionShape.convexHull(
            MeshForgeExtractors.extractPositions(mesh),
            MeshForgeExtractors.extractIndices(mesh)
        );
    }

    public static CollisionShape fittedCapsule(PackedMesh mesh) {
        float[] p = MeshForgeExtractors.extractPositions(mesh);
        if (p.length < 3) {
            return CollisionShape.capsule(0.1f, 0f);
        }
        float minX = Float.POSITIVE_INFINITY, minY = Float.POSITIVE_INFINITY, minZ = Float.POSITIVE_INFINITY;
        float maxX = Float.NEGATIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;
        for (int i = 0; i < p.length; i += 3) {
            minX = java.lang.Math.min(minX, p[i]);
            minY = java.lang.Math.min(minY, p[i + 1]);
            minZ = java.lang.Math.min(minZ, p[i + 2]);
            maxX = java.lang.Math.max(maxX, p[i]);
            maxY = java.lang.Math.max(maxY, p[i + 1]);
            maxZ = java.lang.Math.max(maxZ, p[i + 2]);
        }
        float extentX = maxX - minX;
        float extentY = maxY - minY;
        float extentZ = maxZ - minZ;
        float radius = java.lang.Math.max(0.01f, java.lang.Math.max(extentX, extentZ) * 0.5f);
        float height = java.lang.Math.max(0f, extentY - 2f * radius);
        return CollisionShape.capsule(radius, height);
    }
}
