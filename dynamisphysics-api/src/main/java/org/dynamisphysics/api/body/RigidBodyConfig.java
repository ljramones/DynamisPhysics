package org.dynamisphysics.api.body;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.Objects;

public record RigidBodyConfig(
    CollisionShape shape,
    float mass,
    Matrix4f worldTransform,
    Vector3f linearVelocity,
    Vector3f angularVelocity,
    PhysicsMaterial material,
    BodyMode mode,
    int layer,
    int collidesWith,
    boolean ccd,
    boolean isSensor,
    float gravityScale,
    Object userData
) {
    public static Builder builder(CollisionShape shape, float mass) {
        return new Builder(shape, mass);
    }

    public static final class Builder {
        private final CollisionShape shape;
        private final float mass;
        private Matrix4f worldTransform = new Matrix4f().identity();
        private Vector3f linearVelocity = new Vector3f();
        private Vector3f angularVelocity = new Vector3f();
        private PhysicsMaterial material = PhysicsMaterial.DEFAULT;
        private BodyMode mode = BodyMode.DYNAMIC;
        private int layer = 1;
        private int collidesWith = -1;
        private boolean ccd;
        private boolean isSensor;
        private float gravityScale = 1.0f;
        private Object userData;

        private Builder(CollisionShape shape, float mass) {
            this.shape = Objects.requireNonNull(shape, "shape");
            this.mass = mass;
        }

        public Builder worldTransform(Matrix4f value) { this.worldTransform = value; return this; }
        public Builder linearVelocity(Vector3f value) { this.linearVelocity = value; return this; }
        public Builder angularVelocity(Vector3f value) { this.angularVelocity = value; return this; }
        public Builder material(PhysicsMaterial value) { this.material = value; return this; }
        public Builder mode(BodyMode value) { this.mode = value; return this; }
        public Builder layer(int value) { this.layer = value; return this; }
        public Builder collidesWith(int value) { this.collidesWith = value; return this; }
        public Builder ccd(boolean value) { this.ccd = value; return this; }
        public Builder isSensor(boolean value) { this.isSensor = value; return this; }
        public Builder gravityScale(float value) { this.gravityScale = value; return this; }
        public Builder userData(Object value) { this.userData = value; return this; }

        public RigidBodyConfig build() {
            return new RigidBodyConfig(
                shape,
                mass,
                worldTransform != null ? worldTransform : new Matrix4f().identity(),
                linearVelocity != null ? linearVelocity : new Vector3f(),
                angularVelocity != null ? angularVelocity : new Vector3f(),
                material != null ? material : PhysicsMaterial.DEFAULT,
                mode != null ? mode : BodyMode.DYNAMIC,
                layer,
                collidesWith,
                ccd,
                isSensor,
                gravityScale,
                userData
            );
        }
    }
}
