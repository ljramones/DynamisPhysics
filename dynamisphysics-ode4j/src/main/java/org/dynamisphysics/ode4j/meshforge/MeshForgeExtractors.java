package org.dynamisphysics.ode4j.meshforge;

import org.meshforge.core.attr.AttributeKey;
import org.meshforge.core.attr.AttributeSemantic;
import org.meshforge.core.attr.VertexFormat;
import org.meshforge.pack.buffer.PackedMesh;
import org.meshforge.pack.layout.VertexLayout;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public final class MeshForgeExtractors {
    private MeshForgeExtractors() {
    }

    public static float[] extractPositions(PackedMesh mesh) {
        VertexLayout layout = mesh.layout();
        AttributeKey key = new AttributeKey(AttributeSemantic.POSITION, 0);
        VertexLayout.Entry entry = layout.entry(key);
        if (entry == null) {
            throw new IllegalArgumentException("PackedMesh has no POSITION(0) attribute");
        }
        if (entry.format() != VertexFormat.F32x3) {
            throw new IllegalArgumentException("Expected POSITION format F32x3 but was " + entry.format());
        }

        ByteBuffer vb = mesh.vertexBuffer().duplicate().order(ByteOrder.LITTLE_ENDIAN);
        vb.position(0);
        int stride = layout.strideBytes();
        int vertexCount = vb.remaining() / stride;
        float[] out = new float[vertexCount * 3];
        int baseOffset = entry.offsetBytes();
        for (int i = 0; i < vertexCount; i++) {
            int base = i * stride + baseOffset;
            out[i * 3] = vb.getFloat(base);
            out[i * 3 + 1] = vb.getFloat(base + 4);
            out[i * 3 + 2] = vb.getFloat(base + 8);
        }
        return out;
    }

    public static int[] extractIndices(PackedMesh mesh) {
        PackedMesh.IndexBufferView view = mesh.indexBuffer();
        ByteBuffer ib = view.buffer().duplicate().order(ByteOrder.LITTLE_ENDIAN);
        ib.position(0);
        int count = view.indexCount();
        int[] out = new int[count];
        switch (view.type()) {
            case UINT16 -> {
                for (int i = 0; i < count; i++) {
                    out[i] = Short.toUnsignedInt(ib.getShort(i * 2));
                }
            }
            case UINT32 -> {
                for (int i = 0; i < count; i++) {
                    out[i] = ib.getInt(i * 4);
                }
            }
            default -> throw new IllegalArgumentException("Unsupported index type: " + view.type());
        }
        return out;
    }
}
