package org.dynamisphysics.api.shape;

import org.dynamiscollision.shapes.CollisionShape;

/**
 * Lightweight wrapper type for API signatures that need explicit shape intent.
 */
public record CollisionShapeRef(CollisionShape value) {}
