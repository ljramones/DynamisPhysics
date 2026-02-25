package org.dynamisphysics.api.material;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

class TerrainMaterialTagsTest {
    @Test
    void knownTagsMapToCanonicalMaterials() {
        assertSame(PhysicsMaterial.GRASS, TerrainMaterialTags.toPhysicsMaterial("grass"));
        assertSame(PhysicsMaterial.ROCK, TerrainMaterialTags.toPhysicsMaterial("rock"));
        assertSame(PhysicsMaterial.MUD, TerrainMaterialTags.toPhysicsMaterial("mud"));
        assertSame(PhysicsMaterial.SNOW, TerrainMaterialTags.toPhysicsMaterial("snow"));
        assertSame(PhysicsMaterial.SAND, TerrainMaterialTags.toPhysicsMaterial("sand"));
        assertSame(PhysicsMaterial.ASPHALT, TerrainMaterialTags.toPhysicsMaterial("asphalt"));
        assertSame(PhysicsMaterial.WATER, TerrainMaterialTags.toPhysicsMaterial("water"));
    }

    @Test
    void unknownOrBlankFallsBackToDefault() {
        assertEquals(PhysicsMaterial.DEFAULT, TerrainMaterialTags.toPhysicsMaterial("unknown_terrain_tag"));
        assertEquals(PhysicsMaterial.DEFAULT, TerrainMaterialTags.toPhysicsMaterial(""));
        assertEquals(PhysicsMaterial.DEFAULT, TerrainMaterialTags.toPhysicsMaterial(null));
    }
}
