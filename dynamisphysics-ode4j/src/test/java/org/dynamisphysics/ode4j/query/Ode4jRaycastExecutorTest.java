package org.dynamisphysics.ode4j.query;

import org.dynamisphysics.api.query.RaycastResult;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.OdeHelper;
import org.vectrix.core.Vector3f;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jRaycastExecutorTest {
    private DHashSpace space;

    @BeforeEach
    void setUp() {
        OdeHelper.initODE2(0);
        space = OdeHelper.createHashSpace(null);
    }

    @AfterEach
    void tearDown() {
        space.destroy();
        OdeHelper.closeODE();
    }

    @Test
    void raycastFractionRepresentsHitDistanceOverMaxDist() {
        OdeHelper.createPlane(space, 0.0, 1.0, 0.0, 0.0);

        Ode4jRaycastExecutor executor = new Ode4jRaycastExecutor(space);
        Optional<RaycastResult> hit = executor.raycastClosest(
            new Vector3f(0f, 10f, 0f),
            new Vector3f(0f, -1f, 0f),
            20f,
            -1
        );

        assertTrue(hit.isPresent(), "Expected downward ray to hit plane at y=0");
        assertEquals(0.5f, hit.get().fraction(), 1e-3f);
        assertEquals(0f, hit.get().position().y(), 1e-3f);
    }
}
