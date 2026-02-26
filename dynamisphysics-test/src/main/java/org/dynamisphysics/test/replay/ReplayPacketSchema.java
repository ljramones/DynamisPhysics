package org.dynamisphysics.test.replay;

final class ReplayPacketSchema {
    static final String MAGIC = "DYRP";
    static final int CURRENT_SCHEMA_VERSION = 1;
    static final int MIN_SUPPORTED_SCHEMA_VERSION = 1;
    static final int MAX_SUPPORTED_SCHEMA_VERSION = 1;
    static final String FINGERPRINT = "repro-packet-v1";

    private ReplayPacketSchema() {
    }

    static void validate(ReproPacket packet) {
        if (!MAGIC.equals(packet.magic())) {
            throw new IllegalArgumentException(
                "Unsupported replay packet magic: " + packet.magic() + " expected=" + MAGIC
            );
        }
        int schemaVersion = packet.formatVersion();
        if (schemaVersion < MIN_SUPPORTED_SCHEMA_VERSION || schemaVersion > MAX_SUPPORTED_SCHEMA_VERSION) {
            throw new IllegalArgumentException(
                "Unsupported replay packet schemaVersion: " + schemaVersion
                    + " (supported range: " + MIN_SUPPORTED_SCHEMA_VERSION + "-" + MAX_SUPPORTED_SCHEMA_VERSION + ")"
            );
        }
        if (packet.initialSnapshotB64() == null || packet.initialSnapshotB64().isBlank()) {
            throw new IllegalArgumentException("Replay packet missing initialSnapshotB64");
        }
    }
}
