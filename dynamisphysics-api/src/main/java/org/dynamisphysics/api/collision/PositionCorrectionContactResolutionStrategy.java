package org.dynamisphysics.api.collision;

import org.vectrix.core.Vector3d;

/**
 * Physics-owned position-correction resolution strategy.
 *
 * <p>This migrates one additional solver responsibility to Physics-owned seam handling.
 */
public final class PositionCorrectionContactResolutionStrategy<T> implements PhysicsContactResolutionStrategy<T> {

    private final PhysicsContactBodyAdapter<T> bodyAdapter;
    private double correctionPercent = 0.8;
    private double correctionSlop = 0.001;

    public PositionCorrectionContactResolutionStrategy(PhysicsContactBodyAdapter<T> bodyAdapter) {
        if (bodyAdapter == null) {
            throw new IllegalArgumentException("bodyAdapter must not be null");
        }
        this.bodyAdapter = bodyAdapter;
    }

    public void setCorrectionPercent(double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException("correctionPercent must be in [0,1]");
        }
        correctionPercent = value;
    }

    public void setCorrectionSlop(double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException("correctionSlop must be >= 0");
        }
        correctionSlop = value;
    }

    @Override
    public void resolve(DetectedCollisionContact<T> contact, float deltaSeconds) {
        if (contact == null) {
            throw new IllegalArgumentException("contact must not be null");
        }

        T bodyA = contact.bodyA();
        T bodyB = contact.bodyB();
        double invMassA = Math.max(0.0, bodyAdapter.getInverseMass(bodyA));
        double invMassB = Math.max(0.0, bodyAdapter.getInverseMass(bodyB));
        double invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0) {
            return;
        }

        var manifold = contact.manifold().manifold();
        double correctionMagnitude = Math.max(0.0, manifold.penetrationDepth() - correctionSlop)
                * correctionPercent / invMassSum;
        if (correctionMagnitude <= 0.0) {
            return;
        }

        Vector3d correction = new Vector3d(
                manifold.normalX() * correctionMagnitude,
                manifold.normalY() * correctionMagnitude,
                manifold.normalZ() * correctionMagnitude);

        Vector3d positionA = bodyAdapter.getPosition(bodyA);
        Vector3d positionB = bodyAdapter.getPosition(bodyB);
        bodyAdapter.setPosition(bodyA, new Vector3d(
                positionA.x() - correction.x() * invMassA,
                positionA.y() - correction.y() * invMassA,
                positionA.z() - correction.z() * invMassA));
        bodyAdapter.setPosition(bodyB, new Vector3d(
                positionB.x() + correction.x() * invMassB,
                positionB.y() + correction.y() * invMassB,
                positionB.z() + correction.z() * invMassB));
    }
}
