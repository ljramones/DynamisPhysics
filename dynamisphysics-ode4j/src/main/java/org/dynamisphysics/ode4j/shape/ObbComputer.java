package org.dynamisphysics.ode4j.shape;

import org.vectrix.core.Vector3f;

final class ObbComputer {
    private ObbComputer() {}

    record OrientedBoundingBox(
        float halfExtentA,
        float halfExtentB,
        float halfExtentC,
        Vector3f axisA,
        Vector3f axisB,
        Vector3f axisC
    ) {
        float projectExtent(Vector3f axis) {
            return Math.abs(axisA.dot(axis)) * halfExtentA
                + Math.abs(axisB.dot(axis)) * halfExtentB
                + Math.abs(axisC.dot(axis)) * halfExtentC;
        }
    }

    static OrientedBoundingBox compute(float[] positions) {
        float minX = Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;
        float maxX = -Float.MAX_VALUE;
        float maxY = -Float.MAX_VALUE;
        float maxZ = -Float.MAX_VALUE;

        for (int i = 0; i < positions.length; i += 3) {
            float x = positions[i];
            float y = positions[i + 1];
            float z = positions[i + 2];
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
            maxZ = Math.max(maxZ, z);
        }

        return new OrientedBoundingBox(
            (maxX - minX) * 0.5f,
            (maxY - minY) * 0.5f,
            (maxZ - minZ) * 0.5f,
            new Vector3f(1f, 0f, 0f),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(0f, 0f, 1f)
        );
    }
}
