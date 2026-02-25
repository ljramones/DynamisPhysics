package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamiscollision.shapes.ConvexHullCollisionShape;
import org.dynamiscollision.shapes.HeightfieldCollisionShape;
import org.dynamiscollision.shapes.TriangleMeshCollisionShape;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSimpleSpace;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.OdeHelper;
import org.vectrix.affine.Transformf;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toOde;

public final class Ode4jShapeAdapter {
    private Ode4jShapeAdapter() {}

    public static DGeom toGeom(CollisionShape shape, DSpace space) {
        return switch (shape.shapeType()) {
            case SPHERE -> {
                var s = (org.dynamiscollision.shapes.SphereCollisionShape) shape;
                yield OdeHelper.createSphere(space, s.radius());
            }
            case BOX -> {
                var b = (org.dynamiscollision.shapes.BoxCollisionShape) shape;
                yield OdeHelper.createBox(space, b.halfExtentX() * 2.0, b.halfExtentY() * 2.0, b.halfExtentZ() * 2.0);
            }
            case CAPSULE -> {
                var c = (org.dynamiscollision.shapes.CapsuleCollisionShape) shape;
                yield OdeHelper.createCapsule(space, c.radius(), c.height());
            }
            case CYLINDER -> {
                var c = (org.dynamiscollision.shapes.CylinderCollisionShape) shape;
                yield OdeHelper.createCylinder(space, c.radius(), c.height());
            }
            case PLANE -> {
                var p = (org.dynamiscollision.shapes.PlaneCollisionShape) shape;
                yield OdeHelper.createPlane(space, p.normalX(), p.normalY(), p.normalZ(), p.distance());
            }
            case CONVEX_HULL -> {
                ConvexHullCollisionShape h = (ConvexHullCollisionShape) shape;
                DTriMeshData triMeshData = OdeHelper.createTriMeshData();
                triMeshData.build(h.vertices(), h.indices());
                yield OdeHelper.createTriMesh(space, triMeshData);
            }
            case TRIANGLE_MESH -> {
                TriangleMeshCollisionShape m = (TriangleMeshCollisionShape) shape;
                DTriMeshData triMeshData = OdeHelper.createTriMeshData();
                triMeshData.build(m.vertices(), m.indices());
                yield OdeHelper.createTriMesh(space, triMeshData);
            }
            case HEIGHTFIELD -> {
                HeightfieldCollisionShape hf = (HeightfieldCollisionShape) shape;
                DHeightfieldData hfd = OdeHelper.createHeightfieldData();
                hfd.build(
                    hf.heights(), false,
                    hf.worldWidth(), hf.worldDepth(),
                    hf.widthSamples(), hf.depthSamples(),
                    1.0, 0.0, 0.5, false
                );
                hfd.setBounds(-1.0, hf.maxHeight() + 1.0);
                yield OdeHelper.createHeightfield(space, hfd, true);
            }
            case COMPOUND -> {
                CompoundCollisionShape c = (CompoundCollisionShape) shape;
                DSimpleSpace sub = OdeHelper.createSimpleSpace(space);
                for (int i = 0; i < c.childCount(); i++) {
                    DGeom child = toGeom(c.children().get(i), sub);
                    applyLocalTransform(child, c.localTransforms().get(i));
                }
                yield sub;
            }
        };
    }

    private static void applyLocalTransform(DGeom geom, Transformf t) {
        geom.setPosition(toOde(t.translation));
        geom.setQuaternion(toOde(t.rotation));
    }
}
