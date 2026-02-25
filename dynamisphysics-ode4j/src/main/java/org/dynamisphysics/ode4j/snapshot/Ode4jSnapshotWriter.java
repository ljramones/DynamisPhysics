package org.dynamisphysics.ode4j.snapshot;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;

final class Ode4jSnapshotWriter {
    private byte[] buffer;
    private int pos;

    Ode4jSnapshotWriter(int initialCapacity) {
        this.buffer = new byte[Math.max(initialCapacity, 256)];
    }

    void writeByte(int value) {
        ensure(1);
        buffer[pos++] = (byte) value;
    }

    void writeBoolean(boolean value) {
        writeByte(value ? 1 : 0);
    }

    void writeShort(int value) {
        ensure(2);
        buffer[pos++] = (byte) (value & 0xFF);
        buffer[pos++] = (byte) ((value >>> 8) & 0xFF);
    }

    void writeInt(int value) {
        ensure(4);
        buffer[pos++] = (byte) (value & 0xFF);
        buffer[pos++] = (byte) ((value >>> 8) & 0xFF);
        buffer[pos++] = (byte) ((value >>> 16) & 0xFF);
        buffer[pos++] = (byte) ((value >>> 24) & 0xFF);
    }

    void writeFloat(float value) {
        float canonical = canonicalize(value);
        writeInt(Float.floatToIntBits(canonical));
    }

    void writeString(String value) {
        if (value == null) {
            writeInt(-1);
            return;
        }
        byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
        writeInt(bytes.length);
        ensure(bytes.length);
        System.arraycopy(bytes, 0, buffer, pos, bytes.length);
        pos += bytes.length;
    }

    byte[] toByteArray() {
        return Arrays.copyOf(buffer, pos);
    }

    private void ensure(int needed) {
        int required = pos + needed;
        if (required <= buffer.length) {
            return;
        }
        int next = buffer.length;
        while (next < required) {
            next <<= 1;
        }
        buffer = Arrays.copyOf(buffer, next);
    }

    private static float canonicalize(float value) {
        if (!Float.isFinite(value)) {
            return value;
        }
        return java.lang.Math.round(value * 1_000_000f) / 1_000_000f;
    }
}
