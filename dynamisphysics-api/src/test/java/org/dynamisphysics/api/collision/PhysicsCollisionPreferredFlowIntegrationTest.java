package org.dynamisphysics.api.collision;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.broadphase.SweepAndPrune3D;
import org.dynamiscollision.contact.ContactGenerator3D;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.filtering.CollisionFilter;
import org.dynamiscollision.world.CollisionResponder3D;
import org.dynamiscollision.world.CollisionWorld3D;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3d;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsCollisionPreferredFlowIntegrationTest {

    @Test
    void representativeFlowUsesConfiguredPhysicsPreferredPolicies() {
        Body bodyA = new Body("a", new Vector3d(0.0, 0.0, 0.0), new Vector3d(1.0, 0.0, 0.0), 0.5, 1.0, 0.2);
        Body bodyB = new Body("b", new Vector3d(0.8, 0.0, 0.0), new Vector3d(-1.0, 0.0, 0.0), 0.5, 1.0, 0.2);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::aabb,
                body -> CollisionFilter.DEFAULT,
                (left, right) -> ContactGenerator3D.generate(left.aabb(), right.aabb()));
        CountingWarmStartPolicy<Body> warmStartPolicy = new CountingWarmStartPolicy<>(new PhysicsWarmStartImpulse(0.3, 0.0));
        AtomicInteger fallbackCalls = new AtomicInteger(0);
        CollisionResponder3D<Body> fallback = event -> fallbackCalls.incrementAndGet();
        BodyContactAdapter adapter = new BodyContactAdapter();
        PhysicsPreferredCollisionResponder<Body> preferred = new PhysicsPreferredCollisionResponder<>(
                List.of(
                        new PositionCorrectionContactResolutionStrategy<>(adapter),
                        new NormalImpulseContactResolutionStrategy<>(adapter),
                        new PolicyBackedWarmStartApplicationStrategy<>(adapter, warmStartPolicy)),
                fallback,
                warmStartPolicy,
                (contact, loadedBefore) -> new PhysicsWarmStartImpulse(loadedBefore.normalImpulse() + 1.0, loadedBefore.tangentImpulse()),
                (event, base) -> base,
                event -> event.responseEnabled() && event.type() != CollisionEventType.EXIT && event.manifold() != null);
        world.setResponder(preferred);

        Vector3d startPosA = bodyA.position;
        Vector3d startPosB = bodyB.position;
        double startVelA = bodyA.velocity.x();
        double startVelB = bodyB.velocity.x();

        var events = world.update(List.of(bodyA, bodyB));

        assertEquals(1, events.size());
        assertEquals(CollisionEventType.ENTER, events.get(0).type());
        assertEquals(0, fallbackCalls.get());
        assertTrue(bodyA.position.x() < startPosA.x(), "Position correction should move bodyA along seam-preferred path");
        assertTrue(bodyB.position.x() > startPosB.x(), "Position correction should move bodyB along seam-preferred path");
        assertTrue(bodyA.velocity.x() < startVelA, "Normal impulse/warm-start should affect velocity A");
        assertTrue(bodyB.velocity.x() > startVelB, "Normal impulse/warm-start should affect velocity B");
        assertEquals(2, warmStartPolicy.loadCalls.get(), "Warm-start policy should be loaded during seam path");
        assertEquals(1, warmStartPolicy.storeCalls.get(), "Warm-start persistence update should store once after seam resolution");
        assertEquals(1.3, warmStartPolicy.lastStored.normalImpulse(), 1e-9);
    }

    @Test
    void representativeFlowCanStillRouteToFallbackThroughSelectionPolicy() {
        Body bodyA = new Body("a", new Vector3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0), 0.5, 1.0, 0.0);
        Body bodyB = new Body("b", new Vector3d(0.8, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0), 0.5, 1.0, 0.0);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::aabb,
                body -> CollisionFilter.DEFAULT,
                (left, right) -> ContactGenerator3D.generate(left.aabb(), right.aabb()));
        AtomicInteger strategyCalls = new AtomicInteger(0);
        AtomicInteger fallbackCalls = new AtomicInteger(0);
        CollisionResponder3D<Body> fallback = event -> fallbackCalls.incrementAndGet();
        PhysicsPreferredCollisionResponder<Body> preferred = new PhysicsPreferredCollisionResponder<>(
                List.of((contact, dt) -> strategyCalls.incrementAndGet()),
                fallback,
                null,
                (contact, loaded) -> loaded,
                (event, base) -> base,
                event -> false);
        world.setResponder(preferred);

        var events = world.update(List.of(bodyA, bodyB));

        assertEquals(1, events.size());
        assertEquals(CollisionEventType.ENTER, events.get(0).type());
        assertEquals(0, strategyCalls.get());
        assertEquals(1, fallbackCalls.get());
    }

    private static final class Body {
        private final String id;
        private Vector3d position;
        private Vector3d velocity;
        private final double halfExtent;
        private final double inverseMass;
        private final double restitution;

        private Body(String id, Vector3d position, Vector3d velocity, double halfExtent, double inverseMass, double restitution) {
            this.id = id;
            this.position = position;
            this.velocity = velocity;
            this.halfExtent = halfExtent;
            this.inverseMass = inverseMass;
            this.restitution = restitution;
        }

        private Aabb aabb() {
            return new Aabb(
                    position.x() - halfExtent, position.y() - halfExtent, position.z() - halfExtent,
                    position.x() + halfExtent, position.y() + halfExtent, position.z() + halfExtent);
        }

        @Override
        public String toString() {
            return id;
        }
    }

    private static final class BodyContactAdapter implements PhysicsContactBodyAdapter<Body> {
        @Override
        public Vector3d getPosition(Body body) {
            return body.position;
        }

        @Override
        public void setPosition(Body body, Vector3d position) {
            body.position = position;
        }

        @Override
        public Vector3d getVelocity(Body body) {
            return body.velocity;
        }

        @Override
        public void setVelocity(Body body, Vector3d velocity) {
            body.velocity = velocity;
        }

        @Override
        public double getInverseMass(Body body) {
            return body.inverseMass;
        }

        @Override
        public double getRestitution(Body body) {
            return body.restitution;
        }
    }

    private static final class CountingWarmStartPolicy<T> implements PhysicsWarmStartCachePolicy<T> {
        private final PhysicsWarmStartImpulse loadValue;
        private final AtomicInteger loadCalls = new AtomicInteger(0);
        private final AtomicInteger storeCalls = new AtomicInteger(0);
        private PhysicsWarmStartImpulse lastStored = PhysicsWarmStartImpulse.ZERO;

        private CountingWarmStartPolicy(PhysicsWarmStartImpulse loadValue) {
            this.loadValue = loadValue;
        }

        @Override
        public PhysicsWarmStartImpulse load(DetectedCollisionContact<T> contact) {
            loadCalls.incrementAndGet();
            return loadValue;
        }

        @Override
        public void store(DetectedCollisionContact<T> contact, PhysicsWarmStartImpulse impulse) {
            storeCalls.incrementAndGet();
            lastStored = impulse;
        }

        @Override
        public void onCollisionEvent(org.dynamiscollision.events.CollisionEvent<T> event) {
            // no-op for focused representative flow test
        }
    }
}
