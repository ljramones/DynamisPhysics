package org.dynamisphysics.test;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.test.scene.SceneFactory;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class SceneFactoryTest {
    @Test void emptyWorldHasNoBodies() { var world = SceneFactory.empty(); assertEquals(0, world.spawnedBodyCount()); }
    @Test void flatPlaneWithSphereSpawnsTwoBodies() { var world = SceneFactory.flatPlaneWithSphere(0.5f, 10f); assertEquals(2, world.spawnedBodyCount()); }

    @Test
    void flatPlaneWithSphereHasOneDynamicOneStatic() {
        var world = SceneFactory.flatPlaneWithSphere(0.5f, 10f);
        var configs = world.spawnedConfigs();
        long dynamic = configs.stream().filter(c -> c.mode() == BodyMode.DYNAMIC).count();
        long statik = configs.stream().filter(c -> c.mode() == BodyMode.STATIC).count();
        assertEquals(1, dynamic);
        assertEquals(1, statik);
    }

    @Test void stackedBoxesSpawnsCorrectCount() { var world = SceneFactory.stackedBoxes(5, 1f); assertEquals(5, world.spawnedBodyCount()); }
    @Test void rampAndSphereSpawnsTwoBodies() { var world = SceneFactory.rampAndSphere(); assertEquals(2, world.spawnedBodyCount()); }
    @Test void twoBodiesSpawnsTwoBodies() { var world = SceneFactory.twoBodies(2f); assertEquals(2, world.spawnedBodyCount()); }

    @Test
    void groundWithCharacterSpawnsGroundAndCharacter() {
        var world = SceneFactory.groundWithCharacter();
        assertEquals(1, world.spawnedBodyCount());
        assertEquals(1, world.spawnedCharCount());
    }
}
