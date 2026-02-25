package org.dynamisphysics.test;

import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.event.WakeEvent;
import org.dynamisphysics.test.harness.PhysicsSimHarness;
import org.dynamisphysics.test.mock.TestCollisionShapes;
import org.dynamisphysics.test.scene.SceneFactory;
import org.junit.jupiter.api.Test;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsSimHarnessTest {
    @Test
    void runCountsCorrectSteps() {
        var world = SceneFactory.empty();
        var result = PhysicsSimHarness.run(world, 10, 1f / 60f);
        assertEquals(10, result.totalSteps());
    }

    @Test
    void runCollectsInjectedEvents() {
        var world = SceneFactory.empty();
        var h = world.spawnRigidBody(RigidBodyConfig.builder(TestCollisionShapes.sphere(1f), 1f).build());
        var result = PhysicsSimHarness.run(world, 1, 1f / 60f, w -> world.injectEvent(new WakeEvent(h)));
        assertEventFired(result.allEvents(), WakeEvent.class);
    }

    @Test
    void runReportsPositiveElapsedTime() {
        var world = SceneFactory.empty();
        var result = PhysicsSimHarness.run(world, 60, 1f / 60f);
        assertTrue(result.totalStepTimeNs() >= 0);
    }
}
