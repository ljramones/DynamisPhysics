package org.dynamisphysics.api.collision;

import org.dynamisengine.vectrix.core.Vector3d;

/**
 * Physics-owned normal-impulse velocity resolution strategy.
 *
 * <p>This migrates one solver responsibility to a Physics-owned path while legacy collision-side
 * solvers remain as compatibility fallback.
 */
public final class NormalImpulseContactResolutionStrategy<T> implements PhysicsContactResolutionStrategy<T> {

    private final PhysicsContactBodyAdapter<T> bodyAdapter;

    public NormalImpulseContactResolutionStrategy(PhysicsContactBodyAdapter<T> bodyAdapter) {
        if (bodyAdapter == null) {
            throw new IllegalArgumentException("bodyAdapter must not be null");
        }
        this.bodyAdapter = bodyAdapter;
    }

    @Override
    public void resolve(DetectedCollisionContact<T> contact, float deltaSeconds) {
        if (contact == null) {
            throw new IllegalArgumentException("contact must not be null");
        }

        T bodyA = contact.bodyA();
        T bodyB = contact.bodyB();
        Vector3d velocityA = bodyAdapter.getVelocity(bodyA);
        Vector3d velocityB = bodyAdapter.getVelocity(bodyB);

        double invMassA = Math.max(0.0, bodyAdapter.getInverseMass(bodyA));
        double invMassB = Math.max(0.0, bodyAdapter.getInverseMass(bodyB));
        double invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0) {
            return;
        }

        var manifold = contact.manifold().manifold();
        Vector3d normal = new Vector3d(manifold.normalX(), manifold.normalY(), manifold.normalZ());
        Vector3d relativeVelocity = sub(velocityB, velocityA);
        double velocityAlongNormal = dot(relativeVelocity, normal);
        if (velocityAlongNormal > 0.0) {
            return;
        }

        double restitution = Math.min(
                clamp01(bodyAdapter.getRestitution(bodyA)),
                clamp01(bodyAdapter.getRestitution(bodyB)));

        double impulseScalar = -(1.0 + restitution) * velocityAlongNormal / invMassSum;
        Vector3d impulse = scale(normal, impulseScalar);
        bodyAdapter.setVelocity(bodyA, sub(velocityA, scale(impulse, invMassA)));
        bodyAdapter.setVelocity(bodyB, add(velocityB, scale(impulse, invMassB)));
    }

    private static double clamp01(double value) {
        if (value < 0.0) {
            return 0.0;
        }
        return Math.min(1.0, value);
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
