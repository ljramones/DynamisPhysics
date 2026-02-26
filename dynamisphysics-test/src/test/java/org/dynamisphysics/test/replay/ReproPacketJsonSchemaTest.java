package org.dynamisphysics.test.replay;

import org.dynamisphysics.api.config.PhysicsBackend;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ReproPacketJsonSchemaTest {
    @Test
    void roundTripPreservesMagicAndVersion() {
        ReproPacket packet = samplePacket();
        String json = ReproPacketJson.toJson(packet);
        ReproPacket parsed = ReproPacketJson.fromJson(json);
        assertEquals(ReproPacket.MAGIC, parsed.magic());
        assertEquals(ReproPacket.FORMAT_VERSION, parsed.formatVersion());
    }

    @Test
    void rejectsWrongSchemaVersion() {
        ReproPacket packet = samplePacket();
        String json = ReproPacketJson.toJson(packet).replace(
            "\"formatVersion\" : " + ReproPacket.FORMAT_VERSION,
            "\"formatVersion\" : " + (ReproPacket.FORMAT_VERSION + 1)
        );
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () -> ReproPacketJson.fromJson(json));
        assertTrue(ex.getMessage().contains("schemaVersion"));
    }

    @Test
    void rejectsWrongMagic() {
        ReproPacket packet = samplePacket();
        String json = ReproPacketJson.toJson(packet).replace(
            "\"magic\" : \"" + ReproPacket.MAGIC + "\"",
            "\"magic\" : \"BAD!\""
        );
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () -> ReproPacketJson.fromJson(json));
        assertTrue(ex.getMessage().contains("magic"));
    }

    private static ReproPacket samplePacket() {
        return new ReproPacket(
            ReproPacket.MAGIC,
            ReproPacket.FORMAT_VERSION,
            "2026-01-01T00:00:00Z",
            "0.3.2-SNAPSHOT",
            PhysicsBackend.ODE4J,
            new ReproPacket.ReproTuning("DETERMINISTIC", true, 1, "MALLOC", 64, 10),
            new ReproPacket.ReproWorldConfig(1f / 60f, 1),
            new ReproPacket.ReproScene("SchemaTest", Map.of("bodies", 1)),
            ReplayValidationMode.STRICT,
            ReplayInvariants.defaults(),
            42L,
            "AQID",
            List.of(),
            List.of()
        );
    }
}
