package org.dynamisengine.physics.api.collision;

import org.dynamisengine.collision.contact.ContactManifold3D;

/**
 * Physics-owned typed view of collision-detection output.
 *
 * <p>Collision detects and produces manifolds; Physics consumes these records and owns
 * integration/resolution decisions.
 */
public record DetectedCollisionContact<T>(
        T bodyA,
        T bodyB,
        ContactManifold3D manifold) {

    public DetectedCollisionContact {
        if (bodyA == null || bodyB == null) {
            throw new IllegalArgumentException("bodyA/bodyB must not be null");
        }
        if (manifold == null) {
            throw new IllegalArgumentException("manifold must not be null");
        }
    }
}
