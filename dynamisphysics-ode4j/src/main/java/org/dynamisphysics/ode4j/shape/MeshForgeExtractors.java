package org.dynamisphysics.ode4j.shape;

import org.meshforge.core.attr.AttributeKey;
import org.meshforge.core.attr.AttributeSemantic;
import org.meshforge.core.attr.VertexFormat;
import org.meshforge.pack.buffer.PackedMesh;
import org.meshforge.pack.layout.VertexLayout;

import java.nio.ByteBuffer;

final class MeshForgeExtractors {
    private MeshForgeExtractors() {}

    static float[] extractPositions(PackedMesh mesh) {
        VertexLayout layout = mesh.layout();
        VertexLayout.Entry posEntry = layout.entry(new AttributeKey(AttributeSemantic.POSITION, 0));
        if (posEntry == null) {
            throw new IllegalArgumentException("PackedMesh has no POSITION set 0 attribute");
        }
        if (posEntry.format() != VertexFormat.F32x3 && posEntry.format() != VertexFormat.F32x4) {
            throw new IllegalArgumentException("Unsupported POSITION format: " + posEntry.format());
        }

        ByteBuffer vb = mesh.vertexBuffer().duplicate().order(mesh.vertexBuffer().order());
        int stride = layout.strideBytes();
        int vertexCount = vb.limit() / stride;
        int offset = posEntry.offsetBytes();

        float[] out = new float[vertexCount * 3];
        for (int i = 0; i < vertexCount; i++) {
            int base = i * stride + offset;
            out[i * 3] = vb.getFloat(base);
            out[i * 3 + 1] = vb.getFloat(base + 4);
            out[i * 3 + 2] = vb.getFloat(base + 8);
        }
        return out;
    }

    static int[] extractIndices(PackedMesh mesh) {
        PackedMesh.IndexBufferView indexView = mesh.indexBuffer();
        if (indexView == null || indexView.indexCount() == 0) {
            return new int[0];
        }
        ByteBuffer ib = indexView.buffer().duplicate().order(indexView.buffer().order());
        int[] out = new int[indexView.indexCount()];

        switch (indexView.type()) {
            case UINT16 -> {
                for (int i = 0; i < out.length; i++) {
                    out[i] = ib.getShort(i * Short.BYTES) & 0xFFFF;
                }
            }
            case UINT32 -> {
                for (int i = 0; i < out.length; i++) {
                    out[i] = ib.getInt(i * Integer.BYTES);
                }
            }
            default -> throw new IllegalArgumentException("Unsupported index type: " + indexView.type());
        }
        return out;
    }

    static int[] sequentialTriangles(float[] positions) {
        int vertexCount = positions.length / 3;
        int triangleVertices = vertexCount - (vertexCount % 3);
        int[] indices = new int[triangleVertices];
        for (int i = 0; i < triangleVertices; i++) {
            indices[i] = i;
        }
        return indices;
    }
}
