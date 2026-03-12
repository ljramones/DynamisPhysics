package org.dynamisphysics.api.collision;

import org.dynamisengine.vectrix.core.Vector3d;

import java.util.function.Function;

/**
 * Physics-owned warm-start application strategy.
 *
 * <p>This migrates warm-start application policy to Physics seam handling while legacy solver
 * flows remain as compatibility fallback.
 */
public final class WarmStartApplicationContactResolutionStrategy<T> implements PhysicsContactResolutionStrategy<T> {

    private final PhysicsContactBodyAdapter<T> bodyAdapter;
    private final Function<DetectedCollisionContact<T>, PhysicsWarmStartImpulse> warmStartLookup;

    public WarmStartApplicationContactResolutionStrategy(
            PhysicsContactBodyAdapter<T> bodyAdapter,
            Function<DetectedCollisionContact<T>, PhysicsWarmStartImpulse> warmStartLookup) {
        if (bodyAdapter == null) {
            throw new IllegalArgumentException("bodyAdapter must not be null");
        }
        if (warmStartLookup == null) {
            throw new IllegalArgumentException("warmStartLookup must not be null");
        }
        this.bodyAdapter = bodyAdapter;
        this.warmStartLookup = warmStartLookup;
    }

    @Override
    public void resolve(DetectedCollisionContact<T> contact, float deltaSeconds) {
        if (contact == null) {
            throw new IllegalArgumentException("contact must not be null");
        }
        PhysicsWarmStartImpulse warmStart = warmStartLookup.apply(contact);
        if (warmStart == null) {
            warmStart = PhysicsWarmStartImpulse.ZERO;
        }
        if (Math.abs(warmStart.normalImpulse()) <= 0.0 && Math.abs(warmStart.tangentImpulse()) <= 0.0) {
            return;
        }

        T bodyA = contact.bodyA();
        T bodyB = contact.bodyB();
        double invMassA = Math.max(0.0, bodyAdapter.getInverseMass(bodyA));
        double invMassB = Math.max(0.0, bodyAdapter.getInverseMass(bodyB));
        if (invMassA + invMassB <= 0.0) {
            return;
        }

        Vector3d velocityA = bodyAdapter.getVelocity(bodyA);
        Vector3d velocityB = bodyAdapter.getVelocity(bodyB);
        var manifold = contact.manifold().manifold();
        Vector3d normal = new Vector3d(manifold.normalX(), manifold.normalY(), manifold.normalZ());
        Vector3d tangent = tangentDirection(sub(velocityB, velocityA), normal);
        Vector3d warmImpulse = add(
                scale(normal, warmStart.normalImpulse()),
                scale(tangent, warmStart.tangentImpulse()));
        bodyAdapter.setVelocity(bodyA, sub(velocityA, scale(warmImpulse, invMassA)));
        bodyAdapter.setVelocity(bodyB, add(velocityB, scale(warmImpulse, invMassB)));
    }

    private static Vector3d tangentDirection(Vector3d relativeVelocity, Vector3d normal) {
        double tangentX = relativeVelocity.x() - normal.x() * dot(relativeVelocity, normal);
        double tangentY = relativeVelocity.y() - normal.y() * dot(relativeVelocity, normal);
        double tangentZ = relativeVelocity.z() - normal.z() * dot(relativeVelocity, normal);
        double tangentLen = Math.sqrt(tangentX * tangentX + tangentY * tangentY + tangentZ * tangentZ);
        if (tangentLen > 1e-9) {
            return new Vector3d(tangentX / tangentLen, tangentY / tangentLen, tangentZ / tangentLen);
        }
        return anyPerpendicular(normal);
    }

    private static Vector3d anyPerpendicular(Vector3d normal) {
        Vector3d axis = Math.abs(normal.x()) < 0.9 ? new Vector3d(1, 0, 0) : new Vector3d(0, 1, 0);
        Vector3d tangent = cross(normal, axis);
        double len = Math.sqrt(dot(tangent, tangent));
        if (len <= 1e-9) {
            return new Vector3d(0, 0, 1);
        }
        return scale(tangent, 1.0 / len);
    }

    private static Vector3d cross(Vector3d a, Vector3d b) {
        return new Vector3d(
                a.y() * b.z() - a.z() * b.y(),
                a.z() * b.x() - a.x() * b.z(),
                a.x() * b.y() - a.y() * b.x());
    }

    private static double dot(Vector3d a, Vector3d b) {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    private static Vector3d add(Vector3d a, Vector3d b) {
        return new Vector3d(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
    }

    private static Vector3d sub(Vector3d a, Vector3d b) {
        return new Vector3d(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
    }

    private static Vector3d scale(Vector3d a, double s) {
        return new Vector3d(a.x() * s, a.y() * s, a.z() * s);
    }
}
