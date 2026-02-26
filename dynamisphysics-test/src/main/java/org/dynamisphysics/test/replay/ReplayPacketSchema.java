package org.dynamisphysics.test.replay;

final class ReplayPacketSchema {
    static final String MAGIC = "DYRP";
    static final int VERSION = 1;
    static final String FINGERPRINT = "repro-packet-v1";

    private ReplayPacketSchema() {
    }

    static void validate(ReproPacket packet) {
        if (!MAGIC.equals(packet.magic())) {
            throw new IllegalArgumentException(
                "Unsupported replay packet magic: " + packet.magic() + " expected=" + MAGIC
            );
        }
        if (packet.formatVersion() != VERSION) {
            throw new IllegalArgumentException(
                "Unsupported replay packet schemaVersion: " + packet.formatVersion() + " expected=" + VERSION
            );
        }
        if (packet.initialSnapshotB64() == null || packet.initialSnapshotB64().isBlank()) {
            throw new IllegalArgumentException("Replay packet missing initialSnapshotB64");
        }
    }
}
