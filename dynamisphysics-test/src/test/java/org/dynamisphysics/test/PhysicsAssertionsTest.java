package org.dynamisphysics.test;

import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.event.SleepEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.test.mock.MockRigidBodyHandle;
import org.dynamisphysics.test.mock.TestCollisionShapes;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertBodyFalling;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertDeterministic;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertMaterialTag;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertPositionNear;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

class PhysicsAssertionsTest {
    @Test
    void assertPositionNearPassesWithinTolerance() {
        var state = new BodyState(new Vector3f(1f, 2f, 3f), new Quaternionf(), new Vector3f(), new Vector3f(), false);
        assertDoesNotThrow(() -> assertPositionNear(state, new Vector3f(1f, 2.01f, 3f), 0.1f));
    }

    @Test
    void assertPositionNearFailsOutsideTolerance() {
        var state = new BodyState(new Vector3f(0f, 0f, 0f), new Quaternionf(), new Vector3f(), new Vector3f(), false);
        assertThrows(AssertionError.class, () -> assertPositionNear(state, new Vector3f(10f, 0f, 0f), 0.1f));
    }

    @Test
    void assertBodyFallingPassesWhenYDecreases() {
        var before = new BodyState(new Vector3f(0f, 10f, 0f), new Quaternionf(), new Vector3f(), new Vector3f(), false);
        var after = new BodyState(new Vector3f(0f, 5f, 0f), new Quaternionf(), new Vector3f(), new Vector3f(), false);
        assertDoesNotThrow(() -> assertBodyFalling(before, after));
    }

    @Test
    void assertBodyFallingFailsWhenYIncreases() {
        var before = new BodyState(new Vector3f(0f, 5f, 0f), new Quaternionf(), new Vector3f(), new Vector3f(), false);
        var after = new BodyState(new Vector3f(0f, 10f, 0f), new Quaternionf(), new Vector3f(), new Vector3f(), false);
        assertThrows(AssertionError.class, () -> assertBodyFalling(before, after));
    }

    @Test
    void assertEventFiredPassesWhenPresent() {
        var h = new MockRigidBodyHandle(RigidBodyConfig.builder(TestCollisionShapes.sphere(1f), 1f).build());
        var events = List.<PhysicsEvent>of(new SleepEvent(h));
        assertDoesNotThrow(() -> assertEventFired(events, SleepEvent.class));
    }

    @Test
    void assertEventFiredFailsWhenAbsent() {
        var events = List.<PhysicsEvent>of();
        assertThrows(AssertionError.class, () -> assertEventFired(events, SleepEvent.class));
    }

    @Test
    void assertDeterministicPassesForIdenticalArrays() {
        byte[] a = {1, 2, 3};
        byte[] b = {1, 2, 3};
        assertDoesNotThrow(() -> assertDeterministic(a, b));
    }

    @Test
    void assertDeterministicFailsForDifferentArrays() {
        byte[] a = {1, 2, 3};
        byte[] b = {1, 2, 4};
        assertThrows(AssertionError.class, () -> assertDeterministic(a, b));
    }

    @Test
    void assertMaterialTagPassesForMatchingTag() {
        assertDoesNotThrow(() -> assertMaterialTag(PhysicsMaterial.GRASS, "grass"));
    }

    @Test
    void assertMaterialTagFailsForWrongTag() {
        assertThrows(AssertionError.class, () -> assertMaterialTag(PhysicsMaterial.GRASS, "ice"));
    }
}
