package org.dynamisphysics.test.replay;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import java.io.IOException;

public final class ReproPacketJson {
    private static final ObjectMapper MAPPER = new ObjectMapper()
        .disable(SerializationFeature.WRITE_DATES_AS_TIMESTAMPS)
        .disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);

    private ReproPacketJson() {}

    public static String toJson(ReproPacket packet) {
        try {
            ReplayPacketSchema.validate(packet);
            return MAPPER.writerWithDefaultPrettyPrinter().writeValueAsString(packet);
        } catch (JsonProcessingException e) {
            throw new IllegalArgumentException("Failed to serialize repro packet", e);
        }
    }

    public static ReproPacket fromJson(String json) {
        try {
            ReproPacket packet = MAPPER.readValue(json, ReproPacket.class);
            ReplayPacketSchema.validate(packet);
            return packet;
        } catch (IOException e) {
            throw new IllegalArgumentException("Failed to parse repro packet JSON", e);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("Invalid replay packet schema: " + e.getMessage(), e);
        }
    }
}
