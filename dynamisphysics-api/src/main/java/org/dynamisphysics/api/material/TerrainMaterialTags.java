package org.dynamisphysics.api.material;

import java.util.Locale;

/**
 * Maps terrain material tags/ids to canonical {@link PhysicsMaterial} presets.
 */
public final class TerrainMaterialTags {
    private TerrainMaterialTags() {
    }

    public static PhysicsMaterial toPhysicsMaterial(String tag) {
        if (tag == null || tag.isBlank()) {
            return PhysicsMaterial.DEFAULT;
        }
        return switch (tag.toLowerCase(Locale.ROOT)) {
            case "grass" -> PhysicsMaterial.GRASS;
            case "rock" -> PhysicsMaterial.ROCK;
            case "dirt" -> PhysicsMaterial.DIRT;
            case "sand" -> PhysicsMaterial.SAND;
            case "snow" -> PhysicsMaterial.SNOW;
            case "mud" -> PhysicsMaterial.MUD;
            case "ice" -> PhysicsMaterial.ICE;
            case "asphalt" -> PhysicsMaterial.ASPHALT;
            case "water" -> PhysicsMaterial.WATER;
            case "wood" -> PhysicsMaterial.WOOD;
            case "metal" -> PhysicsMaterial.METAL;
            case "rubber" -> PhysicsMaterial.RUBBER;
            default -> PhysicsMaterial.DEFAULT;
        };
    }
}
