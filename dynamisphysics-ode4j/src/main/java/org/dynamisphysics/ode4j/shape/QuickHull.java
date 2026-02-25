package org.dynamisphysics.ode4j.shape;

import java.util.Arrays;

final class QuickHull {
    private QuickHull() {}

    static float[] compute(float[] positions) {
        // Step 4 fallback: pass-through vertices until a dedicated hull op is needed.
        return Arrays.copyOf(positions, positions.length);
    }

    static int[] hullIndices(float[] hullPositions, int[] sourceIndices) {
        if (sourceIndices != null && sourceIndices.length >= 3) {
            return Arrays.copyOf(sourceIndices, sourceIndices.length);
        }
        return MeshForgeExtractors.sequentialTriangles(hullPositions);
    }
}
