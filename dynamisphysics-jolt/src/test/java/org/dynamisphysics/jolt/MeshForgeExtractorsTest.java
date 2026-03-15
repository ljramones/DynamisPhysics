package org.dynamisphysics.jolt;

import org.dynamisphysics.jolt.meshforge.MeshForgeExtractors;
import org.junit.jupiter.api.Test;
import org.dynamisengine.meshforge.api.Meshes;
import org.dynamisengine.meshforge.core.attr.AttributeSemantic;
import org.dynamisengine.meshforge.core.attr.VertexFormat;
import org.dynamisengine.meshforge.core.attr.VertexSchema;
import org.dynamisengine.meshforge.pack.buffer.PackedMesh;
import org.dynamisengine.meshforge.pack.packer.MeshPacker;
import org.dynamisengine.meshforge.pack.spec.PackSpec;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MeshForgeExtractorsTest {
    @Test
    void extractIndicesSupportsUint16() {
        PackedMesh mesh = MeshPacker.pack(
            quadMeshData(),
            baseSpecBuilder().indexPolicy(PackSpec.IndexPolicy.AUTO_16_IF_POSSIBLE).build()
        );
        assertEquals(PackedMesh.IndexType.UINT16, mesh.indexBuffer().type());
        int[] indices = MeshForgeExtractors.extractIndices(mesh);
        assertEquals(mesh.indexBuffer().indexCount(), indices.length);
        assertTrue(indices.length > 0);
    }

    @Test
    void extractIndicesSupportsUint32() {
        PackedMesh mesh = MeshPacker.pack(
            quadMeshData(),
            baseSpecBuilder().indexPolicy(PackSpec.IndexPolicy.FORCE_32).build()
        );
        assertEquals(PackedMesh.IndexType.UINT32, mesh.indexBuffer().type());
        int[] indices = MeshForgeExtractors.extractIndices(mesh);
        assertEquals(mesh.indexBuffer().indexCount(), indices.length);
        assertTrue(indices.length > 0);
    }

    @Test
    void extractPositionsReadsPositionStream() {
        PackedMesh mesh = MeshPacker.pack(quadMeshData(), baseSpecBuilder().build());
        float[] positions = MeshForgeExtractors.extractPositions(mesh);
        assertTrue(positions.length >= 9);
        for (float p : positions) {
            assertTrue(Float.isFinite(p), "non-finite position");
        }
    }

    private static org.dynamisengine.meshforge.core.mesh.MeshData quadMeshData() {
        VertexSchema schema = VertexSchema.builder()
            .add(AttributeSemantic.POSITION, VertexFormat.F32x3)
            .build();
        float[] positions = {
            -1f, 0f, -1f,
             1f, 0f, -1f,
             1f, 0f,  1f,
            -1f, 0f,  1f
        };
        int[] indices = {
            0, 1, 2,
            0, 2, 3,
            2, 1, 0,
            3, 2, 0
        };
        return Meshes.writer(schema, 4, indices.length)
            .positions(positions)
            .indices(indices)
            .build();
    }

    private static PackSpec.Builder baseSpecBuilder() {
        return PackSpec.builder()
            .target(AttributeSemantic.POSITION, 0, VertexFormat.F32x3)
            .dropUnknownAttributes(true);
    }
}
