package org.dynamisphysics.ode4j.snapshot;

import java.nio.charset.StandardCharsets;

final class Ode4jSnapshotReader {
    private final byte[] buffer;
    private int pos;

    Ode4jSnapshotReader(byte[] buffer) {
        this.buffer = buffer;
    }

    int readByte() {
        require(1);
        return buffer[pos++] & 0xFF;
    }

    boolean readBoolean() {
        return readByte() != 0;
    }

    int readShort() {
        require(2);
        int v0 = buffer[pos++] & 0xFF;
        int v1 = buffer[pos++] & 0xFF;
        return v0 | (v1 << 8);
    }

    int readInt() {
        require(4);
        int v0 = buffer[pos++] & 0xFF;
        int v1 = buffer[pos++] & 0xFF;
        int v2 = buffer[pos++] & 0xFF;
        int v3 = buffer[pos++] & 0xFF;
        return v0 | (v1 << 8) | (v2 << 16) | (v3 << 24);
    }

    float readFloat() {
        return Float.intBitsToFloat(readInt());
    }

    String readString() {
        int len = readInt();
        if (len < 0) {
            return null;
        }
        require(len);
        String s = new String(buffer, pos, len, StandardCharsets.UTF_8);
        pos += len;
        return s;
    }

    boolean hasRemaining() {
        return pos < buffer.length;
    }

    private void require(int count) {
        if (pos + count > buffer.length) {
            throw new IllegalArgumentException("Corrupt snapshot: unexpected EOF");
        }
    }
}
