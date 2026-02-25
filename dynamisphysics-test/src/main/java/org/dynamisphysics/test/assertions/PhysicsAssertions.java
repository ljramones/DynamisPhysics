package org.dynamisphysics.test.assertions;

import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsStats;
import org.vectrix.core.Vector3f;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

public final class PhysicsAssertions {
    private PhysicsAssertions() {}

    public static void assertBodyAtRest(BodyState state, float toleranceMs) {
        assertTrue(state.sleeping() || state.linearVelocity().length() < toleranceMs,
            "Body not at rest: v=" + state.linearVelocity().length());
    }

    public static void assertPositionNear(BodyState state, Vector3f expected, float toleranceMetres) {
        float dist = state.position().distance(expected);
        assertTrue(dist < toleranceMetres,
            "Position " + fmt(state.position()) + " not within " + toleranceMetres +
                "m of " + fmt(expected) + " (dist=" + dist + ")");
    }

    public static void assertBodyFalling(BodyState before, BodyState after) {
        assertTrue(after.position().y() < before.position().y(),
            "Body not falling: y before=" + before.position().y() + " y after=" + after.position().y());
    }

    public static void assertBodyMovingUp(BodyState state) {
        assertTrue(state.linearVelocity().y() > 0f,
            "Body not moving upward: vy=" + state.linearVelocity().y());
    }

    public static void assertBodySleeping(BodyState state) {
        assertTrue(state.sleeping(), "Body not sleeping");
    }

    public static void assertYAbove(BodyState state, float minY) {
        assertTrue(state.position().y() > minY,
            "Body y=" + state.position().y() + " not above " + minY);
    }

    public static void assertYBelow(BodyState state, float maxY) {
        assertTrue(state.position().y() < maxY,
            "Body y=" + state.position().y() + " not below " + maxY);
    }

    public static void assertContactFired(List<PhysicsEvent> events, RigidBodyHandle a, RigidBodyHandle b) {
        assertTrue(events.stream()
                .filter(e -> e instanceof ContactEvent c &&
                    ((c.bodyA() == a && c.bodyB() == b) || (c.bodyA() == b && c.bodyB() == a)))
                .findAny()
                .isPresent(),
            "No ContactEvent between handle " + a + " and handle " + b);
    }

    public static void assertEventFired(List<PhysicsEvent> events, Class<? extends PhysicsEvent> type) {
        assertTrue(events.stream().anyMatch(type::isInstance),
            "No event of type " + type.getSimpleName() + " in list of " + events.size() + " events");
    }

    public static void assertNoContactFired(List<PhysicsEvent> events) {
        assertTrue(events.stream().noneMatch(e -> e instanceof ContactEvent),
            "Unexpected ContactEvent in event list");
    }

    public static void assertEventCount(List<PhysicsEvent> events, Class<? extends PhysicsEvent> type, int expected) {
        long count = events.stream().filter(type::isInstance).count();
        assertEquals(expected, count,
            "Expected " + expected + " events of type " + type.getSimpleName() + " but found " + count);
    }

    public static void assertDeterministic(byte[] snap1, byte[] snap2) {
        assertArrayEquals(snap1, snap2,
            "Physics snapshots not bit-identical — determinism violated");
    }

    public static void assertSnapshotNonEmpty(byte[] snapshot) {
        assertNotNull(snapshot, "Snapshot is null");
        assertTrue(snapshot.length > 0, "Snapshot is empty");
    }

    public static void assertNoTunnelling(List<BodyState> states, float maxDeltaPerStep) {
        for (int i = 1; i < states.size(); i++) {
            float delta = states.get(i).position().distance(states.get(i - 1).position());
            assertTrue(delta < maxDeltaPerStep,
                "Tunnelling at step " + i + ": delta=" + delta + " > max=" + maxDeltaPerStep);
        }
    }

    public static void assertMaterialTag(PhysicsMaterial m, String expectedTag) {
        assertEquals(expectedTag, m.tag(),
            "Expected material tag '" + expectedTag + "' got '" + m.tag() + "'");
    }

    public static void assertFrictionBelow(PhysicsMaterial m, float max) {
        assertTrue(m.friction() < max,
            "Friction " + m.friction() + " not below " + max);
    }

    public static void assertFrictionAbove(PhysicsMaterial m, float min) {
        assertTrue(m.friction() > min,
            "Friction " + m.friction() + " not above " + min);
    }

    public static void assertAlive(RigidBodyHandle h) {
        assertTrue(h.isAlive(), "Handle expected alive but is dead");
    }

    public static void assertDead(RigidBodyHandle h) {
        assertFalse(h.isAlive(), "Handle expected dead but is alive");
    }

    public static void assertStatsBodyCount(PhysicsStats stats, int expected) {
        assertEquals(expected, stats.bodyCount(),
            "Expected bodyCount=" + expected + " got " + stats.bodyCount());
    }

    public static void assertStepTimePlausible(PhysicsStats stats) {
        assertTrue(stats.stepTimeMs() >= 0f,
            "stepTimeMs is negative: " + stats.stepTimeMs());
        assertTrue(stats.stepTimeMs() < 5000f,
            "stepTimeMs implausibly large: " + stats.stepTimeMs());
    }

    private static String fmt(Vector3f v) {
        return String.format("(%.3f, %.3f, %.3f)", v.x(), v.y(), v.z());
    }
}
