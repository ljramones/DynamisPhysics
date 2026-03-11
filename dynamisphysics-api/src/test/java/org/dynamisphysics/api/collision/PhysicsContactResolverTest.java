package org.dynamisphysics.api.collision;

import org.dynamiscollision.contact.ContactManifold3D;
import org.dynamiscollision.contact.ContactPoint3D;
import org.dynamiscollision.contact.ContactSolver3D;
import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.pipeline.CollisionPair;
import org.dynamiscollision.contact.WarmStartImpulse;
import org.dynamiscollision.world.CollisionResponder3D;
import org.dynamiscollision.world.RigidBodyAdapter3D;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3d;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

    @Test
    void normalImpulseStrategyMatchesLegacySolverForSimpleHeadOnContact() {
        TestBody physicsA = new TestBody(new Vector3d(5.0, 0.0, 0.0), 1.0, 0.3);
        TestBody physicsB = new TestBody(new Vector3d(-1.0, 0.0, 0.0), 1.0, 0.3);
        TestBody legacyA = physicsA.copy();
        TestBody legacyB = physicsB.copy();

        var manifold = sampleManifold();

        PhysicsContactResolver<TestBody> physicsResolver = new PhysicsContactResolver<>();
        physicsResolver.resolve(
                List.of(new DetectedCollisionContact<>(physicsA, physicsB, manifold)),
                1f / 60f,
                new NormalImpulseContactResolutionStrategy<>(new TestPhysicsAdapter()));

        ContactSolver3D<TestBody> legacySolver = new ContactSolver3D<>(new TestLegacyAdapter());
        legacySolver.solveVelocity(new CollisionPair<>(legacyA, legacyB), manifold, WarmStartImpulse.ZERO);

        assertEquals(legacyA.velocity().x(), physicsA.velocity().x(), 1e-9);
        assertEquals(legacyA.velocity().y(), physicsA.velocity().y(), 1e-9);
        assertEquals(legacyA.velocity().z(), physicsA.velocity().z(), 1e-9);
        assertEquals(legacyB.velocity().x(), physicsB.velocity().x(), 1e-9);
        assertEquals(legacyB.velocity().y(), physicsB.velocity().y(), 1e-9);
        assertEquals(legacyB.velocity().z(), physicsB.velocity().z(), 1e-9);
    }

    @Test
    void positionCorrectionStrategyMatchesLegacySolverForSimplePenetration() {
        TestBody physicsA = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        TestBody physicsB = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        TestBody legacyA = physicsA.copy();
        TestBody legacyB = physicsB.copy();

        var manifold = sampleManifold();

        PhysicsContactResolver<TestBody> physicsResolver = new PhysicsContactResolver<>();
        PositionCorrectionContactResolutionStrategy<TestBody> strategy =
                new PositionCorrectionContactResolutionStrategy<>(new TestPhysicsAdapter());
        physicsResolver.resolve(
                List.of(new DetectedCollisionContact<>(physicsA, physicsB, manifold)),
                1f / 60f,
                strategy);

        ContactSolver3D<TestBody> legacySolver = new ContactSolver3D<>(new TestLegacyAdapter());
        legacySolver.solvePosition(new CollisionPair<>(legacyA, legacyB), manifold);

        assertEquals(legacyA.position().x(), physicsA.position().x(), 1e-9);
        assertEquals(legacyA.position().y(), physicsA.position().y(), 1e-9);
        assertEquals(legacyA.position().z(), physicsA.position().z(), 1e-9);
        assertEquals(legacyB.position().x(), physicsB.position().x(), 1e-9);
        assertEquals(legacyB.position().y(), physicsB.position().y(), 1e-9);
        assertEquals(legacyB.position().z(), physicsB.position().z(), 1e-9);
    }

    @Test
    void warmStartApplicationStrategyMatchesLegacyWarmStartEffect() {
        TestBody physicsA = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        TestBody physicsB = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        TestBody legacyA = physicsA.copy();
        TestBody legacyB = physicsB.copy();

        var manifold = sampleManifold();
        var warmStart = new PhysicsWarmStartImpulse(0.7, 0.2);

        PhysicsContactResolver<TestBody> physicsResolver = new PhysicsContactResolver<>();
        physicsResolver.resolve(
                List.of(new DetectedCollisionContact<>(physicsA, physicsB, manifold)),
                1f / 60f,
                new WarmStartApplicationContactResolutionStrategy<>(new TestPhysicsAdapter(), c -> warmStart));

        ContactSolver3D<TestBody> legacySolver = new ContactSolver3D<>(new TestLegacyAdapter());
        legacySolver.solveVelocity(
                new CollisionPair<>(legacyA, legacyB),
                manifold,
                new WarmStartImpulse(warmStart.normalImpulse(), warmStart.tangentImpulse()));

        assertEquals(legacyA.velocity().x(), physicsA.velocity().x(), 1e-9);
        assertEquals(legacyA.velocity().y(), physicsA.velocity().y(), 1e-9);
        assertEquals(legacyA.velocity().z(), physicsA.velocity().z(), 1e-9);
        assertEquals(legacyB.velocity().x(), physicsB.velocity().x(), 1e-9);
        assertEquals(legacyB.velocity().y(), physicsB.velocity().y(), 1e-9);
        assertEquals(legacyB.velocity().z(), physicsB.velocity().z(), 1e-9);
    }

    @Test
    void preferredResponderUsesPhysicsSeamBeforeLegacyFallback() {
        TestBody bodyA = new TestBody(new Vector3d(5.0, 0.0, 0.0), 1.0, 0.3);
        TestBody bodyB = new TestBody(new Vector3d(-1.0, 0.0, 0.0), 1.0, 0.3);
        AtomicInteger fallbackCalls = new AtomicInteger(0);
        CollisionResponder3D<TestBody> fallback = event -> fallbackCalls.incrementAndGet();
        var preferred = new PhysicsPreferredCollisionResponder<>(
                List.of(new NormalImpulseContactResolutionStrategy<>(new TestPhysicsAdapter())),
                fallback);

        var resolvableEvent = new CollisionEvent<>(
                new CollisionPair<>(bodyA, bodyB),
                CollisionEventType.ENTER,
                true,
                sampleManifold());

        preferred.resolve(resolvableEvent);

        assertEquals(0, fallbackCalls.get(), "Fallback should not run when Physics seam resolves");
        assertTrue(bodyA.velocity().x() < 5.0);
        assertTrue(bodyB.velocity().x() > -1.0);
    }

    @Test
    void preferredResponderFallsBackWhenPhysicsSeamDoesNotApply() {
        TestBody bodyA = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        TestBody bodyB = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        AtomicInteger fallbackCalls = new AtomicInteger(0);
        CollisionResponder3D<TestBody> fallback = event -> fallbackCalls.incrementAndGet();
        var preferred = new PhysicsPreferredCollisionResponder<>(
                List.of(new PositionCorrectionContactResolutionStrategy<>(new TestPhysicsAdapter())),
                fallback);

        var nonResolvableEvent = new CollisionEvent<>(
                new CollisionPair<>(bodyA, bodyB),
                CollisionEventType.EXIT,
                true,
                sampleManifold());

        preferred.resolve(nonResolvableEvent);

        assertEquals(1, fallbackCalls.get(), "Fallback should run when seam does not apply");
    }

    @Test
    void preferredResponderUsesPhysicsWarmStartPolicyAndInvalidatesOnExit() {
        TestBody bodyA = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        TestBody bodyB = new TestBody(new Vector3d(0.0, 0.0, 0.0), 1.0, 0.0);
        AtomicInteger fallbackCalls = new AtomicInteger(0);
        CollisionResponder3D<TestBody> fallback = event -> fallbackCalls.incrementAndGet();

        var manifold = sampleManifold();
        var detected = new DetectedCollisionContact<>(bodyA, bodyB, manifold);
        var warmStartPolicy = new MapBackedWarmStartCachePolicy<TestBody>();
        warmStartPolicy.store(detected, new PhysicsWarmStartImpulse(0.8, 0.0));

        var preferred = new PhysicsPreferredCollisionResponder<>(
                List.of(new PolicyBackedWarmStartApplicationStrategy<>(new TestPhysicsAdapter(), warmStartPolicy)),
                fallback,
                warmStartPolicy);

        var enterEvent = new CollisionEvent<>(
                new CollisionPair<>(bodyA, bodyB),
                CollisionEventType.ENTER,
                true,
                manifold);
        preferred.resolve(enterEvent);
        assertEquals(0, fallbackCalls.get());
        assertTrue(bodyA.velocity().x() < 0.0);
        assertTrue(bodyB.velocity().x() > 0.0);

        bodyA.velocity = new Vector3d(0.0, 0.0, 0.0);
        bodyB.velocity = new Vector3d(0.0, 0.0, 0.0);

        var exitEvent = new CollisionEvent<>(
                new CollisionPair<>(bodyA, bodyB),
                CollisionEventType.EXIT,
                true,
                manifold);
        preferred.resolve(exitEvent);
        assertEquals(1, fallbackCalls.get(), "Exit should use fallback and invalidate warm-start cache");

        preferred.resolve(enterEvent);
        assertEquals(1, fallbackCalls.get(), "Resolvable ENTER should still use seam");
        assertEquals(0.0, bodyA.velocity().x(), 1e-9, "Warm-start should be evicted after EXIT");
        assertEquals(0.0, bodyB.velocity().x(), 1e-9, "Warm-start should be evicted after EXIT");
    }

    private static ContactManifold3D sampleManifold() {
        return new ContactManifold3D(
                new CollisionManifold3D(1.0, 0.0, 0.0, 0.05),
                List.of(new ContactPoint3D(0.0, 0.0, 0.0)));
    }

    private static final class TestBody {
        private Vector3d velocity;
        private Vector3d position;
        private final double inverseMass;
        private final double restitution;
        private final double friction;

        private TestBody(Vector3d velocity, double inverseMass, double restitution) {
            this(velocity, inverseMass, restitution, 0.0, new Vector3d(0.0, 0.0, 0.0));
        }

        private TestBody(Vector3d velocity, double inverseMass, double restitution, double friction, Vector3d position) {
            this.velocity = velocity;
            this.inverseMass = inverseMass;
            this.restitution = restitution;
            this.friction = friction;
            this.position = position;
        }

        private TestBody copy() {
            return new TestBody(
                    new Vector3d(velocity.x(), velocity.y(), velocity.z()),
                    inverseMass,
                    restitution,
                    friction,
                    new Vector3d(position.x(), position.y(), position.z()));
        }

        private Vector3d velocity() {
            return velocity;
        }

        private Vector3d position() {
            return position;
        }
    }

    private static final class TestPhysicsAdapter implements PhysicsContactBodyAdapter<TestBody> {
        @Override
        public Vector3d getPosition(TestBody body) {
            return body.position;
        }

        @Override
        public void setPosition(TestBody body, Vector3d position) {
            body.position = position;
        }

        @Override
        public Vector3d getVelocity(TestBody body) {
            return body.velocity;
        }

        @Override
        public void setVelocity(TestBody body, Vector3d velocity) {
            body.velocity = velocity;
        }

        @Override
        public double getInverseMass(TestBody body) {
            return body.inverseMass;
        }

        @Override
        public double getRestitution(TestBody body) {
            return body.restitution;
        }
    }

    private static final class TestLegacyAdapter implements RigidBodyAdapter3D<TestBody> {
        @Override
        public Vector3d getPosition(TestBody body) {
            return body.position;
        }

        @Override
        public void setPosition(TestBody body, Vector3d position) {
            body.position = position;
        }

        @Override
        public Vector3d getVelocity(TestBody body) {
            return body.velocity;
        }

        @Override
        public void setVelocity(TestBody body, Vector3d velocity) {
            body.velocity = velocity;
        }

        @Override
        public double getInverseMass(TestBody body) {
            return body.inverseMass;
        }

        @Override
        public double getRestitution(TestBody body) {
            return body.restitution;
        }

        @Override
        public double getFriction(TestBody body) {
            return body.friction;
        }
    }
}
