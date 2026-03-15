package org.dynamisengine.physics.api.shape;

import org.dynamisengine.collision.shapes.CollisionShape;

/**
 * Lightweight wrapper type for API signatures that need explicit shape intent.
 */
public record CollisionShapeRef(CollisionShape value) {}
