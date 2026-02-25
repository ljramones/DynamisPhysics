package org.dynamisphysics.ode4j.debug;

import org.vectrix.core.Vector3f;

public final class WireframeBuilder {
    private static final int FLOATS_PER_VERTEX = 7;

    private final float[] vertices;
    private int vertexCount;

    public WireframeBuilder(int maxLines) {
        int maxVerts = Math.max(maxLines, 1) * 2;
        this.vertices = new float[maxVerts * FLOATS_PER_VERTEX];
    }

    public void reset() {
        vertexCount = 0;
    }

    public int addLine(Vector3f a, Vector3f b, float[] rgba) {
        if (vertexCount + 2 > maxVertices()) {
            return 0;
        }
        writeVertex(a, rgba);
        writeVertex(b, rgba);
        return 2;
    }

    public int addCross(Vector3f c, float size, float[] rgba) {
        int written = 0;
        written += addLine(new Vector3f(c.x() - size, c.y(), c.z()), new Vector3f(c.x() + size, c.y(), c.z()), rgba);
        written += addLine(new Vector3f(c.x(), c.y() - size, c.z()), new Vector3f(c.x(), c.y() + size, c.z()), rgba);
        written += addLine(new Vector3f(c.x(), c.y(), c.z() - size), new Vector3f(c.x(), c.y(), c.z() + size), rgba);
        return written;
    }

    public int vertexCount() {
        return vertexCount;
    }

    public int lineCount() {
        return vertexCount / 2;
    }

    public float[] vertices() {
        return vertices;
    }

    public int maxVertices() {
        return vertices.length / FLOATS_PER_VERTEX;
    }

    private void writeVertex(Vector3f p, float[] c) {
        int base = vertexCount * FLOATS_PER_VERTEX;
        vertices[base] = p.x();
        vertices[base + 1] = p.y();
        vertices[base + 2] = p.z();
        vertices[base + 3] = c[0];
        vertices[base + 4] = c[1];
        vertices[base + 5] = c[2];
        vertices[base + 6] = c[3];
        vertexCount++;
    }
}
