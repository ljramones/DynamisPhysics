package org.dynamisengine.physics.api.query;

import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.material.PhysicsMaterial;
import org.dynamisengine.vectrix.core.Vector3f;

/**
 * Raycast hit result.
 *
 * <p>{@code fraction} is normalized hit distance in {@code [0..1]} and is
 * defined as {@code hitDistance / maxDist} for the originating ray query.
 */
public record RaycastResult(
    RigidBodyHandle body,
    Vector3f position,
    Vector3f normal,
    float fraction,
    PhysicsMaterial material,
    int layer
) {}
