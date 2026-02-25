package org.dynamisphysics.jolt.shape;

import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.CapsuleShape;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.CylinderShape;
import com.github.stephengold.joltjni.Float3;
import com.github.stephengold.joltjni.IndexedTriangle;
import com.github.stephengold.joltjni.IndexedTriangleList;
import com.github.stephengold.joltjni.MeshShapeSettings;
import com.github.stephengold.joltjni.MutableCompoundShape;
import com.github.stephengold.joltjni.Plane;
import com.github.stephengold.joltjni.PlaneShape;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.ShapeResult;
import com.github.stephengold.joltjni.Shape;
import com.github.stephengold.joltjni.SphereShape;
import com.github.stephengold.joltjni.Vec3;
import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamiscollision.shapes.ConvexHullCollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.PlaneCollisionShape;
import org.dynamiscollision.shapes.SphereCollisionShape;
import org.dynamiscollision.shapes.TriangleMeshCollisionShape;
import com.github.stephengold.joltjni.readonly.ConstShape;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

public final class JoltShapeAdapter {
    private JoltShapeAdapter() {
    }

    public static ConstShape toJoltShape(CollisionShape shape) {
        return switch (shape.shapeType()) {
            case SPHERE -> new SphereShape(((SphereCollisionShape) shape).radius());
            case BOX -> {
                BoxCollisionShape box = (BoxCollisionShape) shape;
                yield new BoxShape(box.halfExtentX(), box.halfExtentY(), box.halfExtentZ());
            }
            case CAPSULE -> {
                CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
                yield new CapsuleShape(capsule.height() * 0.5f, capsule.radius());
            }
            case CYLINDER -> {
                CylinderCollisionShape cylinder = (CylinderCollisionShape) shape;
                yield new CylinderShape(cylinder.height() * 0.5f, cylinder.radius());
            }
            case PLANE -> {
                PlaneCollisionShape p = (PlaneCollisionShape) shape;
                yield new PlaneShape(new Plane(p.normalX(), p.normalY(), p.normalZ(), p.distance()));
            }
            case CONVEX_HULL -> createConvexHull((ConvexHullCollisionShape) shape);
            case TRIANGLE_MESH -> createTriangleMesh((TriangleMeshCollisionShape) shape);
            case COMPOUND -> {
                CompoundCollisionShape c = (CompoundCollisionShape) shape;
                MutableCompoundShape compound = new MutableCompoundShape();
                Vector3f com = JoltCompoundMassProperties.computeCenterOfMass(c);
                for (int i = 0; i < c.childCount(); i++) {
                    Transformf t = c.localTransforms().get(i);
                    Vector3f local = new Vector3f(t.translation).sub(com);
                    ConstShape child = toJoltShape(c.children().get(i));
                    compound.addShape(
                        new Vec3(local.x(), local.y(), local.z()),
                        new Quat(t.rotation.x(), t.rotation.y(), t.rotation.z(), t.rotation.w()),
                        child
                    );
                }
                yield compound;
            }
            default -> throw new UnsupportedOperationException("Jolt shape not implemented yet: " + shape.shapeType());
        };
    }

    private static ConstShape createConvexHull(ConvexHullCollisionShape shape) {
        float[] vertices = shape.vertices();
        List<Vec3> points = new ArrayList<>(vertices.length / 3);
        for (int i = 0; i < vertices.length; i += 3) {
            points.add(new Vec3(vertices[i], vertices[i + 1], vertices[i + 2]));
        }
        ConvexHullShapeSettings settings = new ConvexHullShapeSettings(points);
        return createFromSettings(settings, "ConvexHullShapeSettings");
    }

    private static ConstShape createTriangleMesh(TriangleMeshCollisionShape shape) {
        float[] vertices = shape.vertices();
        int[] indices = shape.indices();
        Float3[] verts = new Float3[vertices.length / 3];
        for (int i = 0; i < verts.length; i++) {
            verts[i] = new Float3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        }
        IndexedTriangleList tris = new IndexedTriangleList();
        tris.resize(indices.length / 3);
        for (int i = 0; i < indices.length; i += 3) {
            tris.set(i / 3, new IndexedTriangle(indices[i], indices[i + 1], indices[i + 2]));
        }
        MeshShapeSettings settings = new MeshShapeSettings(verts, tris);
        return createFromSettings(settings, "MeshShapeSettings");
    }

    private static ConstShape createFromSettings(com.github.stephengold.joltjni.ShapeSettings settings, String label) {
        ShapeResult result = settings.create();
        if (result.hasError()) {
            throw new IllegalStateException(label + " create failed: " + result.getError());
        }
        return result.get();
    }
}
