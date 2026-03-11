package org.dynamisphysics.api.collision;

import org.dynamiscollision.contact.ContactManifold3D;
import org.dynamiscollision.contact.ContactPoint3D;
import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.pipeline.CollisionPair;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class PhysicsContactResolverTest {

    @Test
    void resolvesDetectedContactsThroughPhysicsStrategy() {
        PhysicsContactResolver<String> resolver = new PhysicsContactResolver<>();
        AtomicInteger invocations = new AtomicInteger(0);

        var contacts = List.of(
                new DetectedCollisionContact<>("a", "b", sampleManifold()),
                new DetectedCollisionContact<>("c", "d", sampleManifold()));

        resolver.resolve(contacts, 1f / 60f, (contact, dt) -> {
            invocations.incrementAndGet();
            assertEquals(1f / 60f, dt);
        });

        assertEquals(2, invocations.get());
    }

    @Test
    void rejectsInvalidInputs() {
        PhysicsContactResolver<String> resolver = new PhysicsContactResolver<>();
        var contacts = List.of(new DetectedCollisionContact<>("a", "b", sampleManifold()));

        assertThrows(IllegalArgumentException.class, () -> resolver.resolve(null, 1f / 60f, (c, dt) -> {}));
        assertThrows(IllegalArgumentException.class, () -> resolver.resolve(contacts, 0f, (c, dt) -> {}));
        assertThrows(IllegalArgumentException.class, () -> resolver.resolve(contacts, 1f / 60f, null));
    }

    @Test
    void resolvesLegacyCollisionEventsThroughPhysicsOwnedSeam() {
        PhysicsContactResolver<String> resolver = new PhysicsContactResolver<>();
        AtomicInteger invocations = new AtomicInteger(0);

        var legacyEvents = List.of(
                new CollisionEvent<>(new CollisionPair<>("a", "b"), CollisionEventType.ENTER, true, sampleManifold()),
                new CollisionEvent<>(new CollisionPair<>("c", "d"), CollisionEventType.EXIT, true, sampleManifold()),
                new CollisionEvent<>(new CollisionPair<>("e", "f"), CollisionEventType.STAY, false, sampleManifold()),
                new CollisionEvent<>(new CollisionPair<>("g", "h"), CollisionEventType.STAY, true, null));

        resolver.resolveFromCollisionEvents(legacyEvents, 1f / 120f, (contact, dt) -> {
            invocations.incrementAndGet();
            assertEquals("a", contact.bodyA());
            assertEquals("b", contact.bodyB());
            assertEquals(1f / 120f, dt);
        });

        assertEquals(1, invocations.get());
    }

    private static ContactManifold3D sampleManifold() {
        return new ContactManifold3D(
                new CollisionManifold3D(1.0, 0.0, 0.0, 0.05),
                List.of(new ContactPoint3D(0.0, 0.0, 0.0)));
    }
}
