package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.ConvexHullCollisionShape;
import org.meshforge.pack.buffer.PackedMesh;

public final class ConvexHullShapeBuilder {
    private ConvexHullShapeBuilder() {}

    public static ConvexHullCollisionShape fromPackedMesh(PackedMesh mesh) {
        float[] positions = MeshForgeExtractors.extractPositions(mesh);
        int[] sourceIndices = MeshForgeExtractors.extractIndices(mesh);
        float[] hull = QuickHull.compute(positions);
        int[] hullIndices = QuickHull.hullIndices(hull, sourceIndices);
        return (ConvexHullCollisionShape) CollisionShape.convexHull(hull, hullIndices);
    }
}
