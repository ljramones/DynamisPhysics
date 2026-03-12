package org.dynamisphysics.api.collision;

import org.dynamisengine.vectrix.core.Vector3d;

/**
 * Physics-owned adapter for body state needed by contact-resolution strategies.
 */
public interface PhysicsContactBodyAdapter<T> {

    Vector3d getPosition(T body);

    void setPosition(T body, Vector3d position);

    Vector3d getVelocity(T body);

    void setVelocity(T body, Vector3d velocity);

    double getInverseMass(T body);

    double getRestitution(T body);
}
