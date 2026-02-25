package org.dynamisphysics.ode4j.body;

import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.SphereCollisionShape;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DRotation;
import org.ode4j.ode.OdeHelper;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toOde;

public final class Ode4jCompoundMassProperties {
    private Ode4jCompoundMassProperties() {}

    public static DMass compute(CompoundCollisionShape compound, float totalMass) {
        DMass total = OdeHelper.createMass();
        total.setZero();
        if (compound.childCount() == 0 || totalMass <= 0f) {
            total.setMass(Math.max(totalMass, 0.0001f));
            return total;
        }

        List<CollisionShape> children = compound.children();
        List<Transformf> transforms = compound.localTransforms();
        List<Float> volumes = computeVolumes(children);
        float volumeSum = volumeSum(volumes);
        Vector3f com = computeCenterOfMass(compound, volumes, volumeSum);

        for (int i = 0; i < children.size(); i++) {
            CollisionShape child = children.get(i);
            Transformf local = transforms.get(i);
            float childMass = totalMass * (volumes.get(i) / volumeSum);
            DMass childMassProps = primitiveOrApproxMass(child, childMass);

            DMatrix3 rot = new DMatrix3();
            DRotation.dRfromQ(rot, toOde(local.rotation));
            childMassProps.rotate(rot);
            childMassProps.translate(
                local.translation.x() - com.x(),
                local.translation.y() - com.y(),
                local.translation.z() - com.z()
            );
            total.add(childMassProps);
        }

        if (total.getMass() > 0.0) {
            // ODE requires body mass COM to be exactly at origin.
            total.setC(new DVector3(0.0, 0.0, 0.0));
            total.adjust(totalMass);
        } else {
            total.setMass(Math.max(totalMass, 0.0001f));
        }
        return total;
    }

    public static Vector3f computeCenterOfMass(CompoundCollisionShape compound) {
        if (compound.childCount() == 0) {
            return new Vector3f();
        }
        List<Float> volumes = computeVolumes(compound.children());
        return computeCenterOfMass(compound, volumes, volumeSum(volumes));
    }

    private static Vector3f computeCenterOfMass(CompoundCollisionShape compound, List<Float> volumes, float volumeSum) {
        if (volumeSum <= 0f) {
            return new Vector3f();
        }
        Vector3f com = new Vector3f();
        for (int i = 0; i < compound.childCount(); i++) {
            float w = volumes.get(i) / volumeSum;
            Transformf t = compound.localTransforms().get(i);
            com.add(t.translation.x() * w, t.translation.y() * w, t.translation.z() * w);
        }
        return com;
    }

    private static List<Float> computeVolumes(List<CollisionShape> children) {
        List<Float> volumes = new ArrayList<>(children.size());
        for (CollisionShape child : children) {
            volumes.add(Math.max(approximateVolume(child), 0.0001f));
        }
        if (volumeSum(volumes) <= 0f) {
            volumes.replaceAll(v -> 1f);
        }
        return volumes;
    }

    private static float volumeSum(List<Float> volumes) {
        float volumeSum = 0f;
        for (float v : volumes) {
            volumeSum += v;
        }
        return volumeSum;
    }

    private static DMass primitiveOrApproxMass(CollisionShape shape, float mass) {
        DMass m = OdeHelper.createMass();
        switch (shape.shapeType()) {
            case SPHERE -> {
                SphereCollisionShape s = (SphereCollisionShape) shape;
                m.setSphereTotal(mass, s.radius());
            }
            case BOX -> {
                BoxCollisionShape b = (BoxCollisionShape) shape;
                m.setBoxTotal(mass, b.halfExtentX() * 2.0, b.halfExtentY() * 2.0, b.halfExtentZ() * 2.0);
            }
            case CAPSULE -> {
                CapsuleCollisionShape c = (CapsuleCollisionShape) shape;
                // ODE capsules are aligned to Z by default.
                m.setCapsuleTotal(mass, 3, c.radius(), c.height());
            }
            case CYLINDER -> {
                CylinderCollisionShape c = (CylinderCollisionShape) shape;
                m.setCylinderTotal(mass, 3, c.radius(), c.height());
            }
            default -> {
                float radius = Ode4jInertiaComputer.halfDiagonal(shape);
                m.setSphereTotal(mass, radius);
            }
        }
        return m;
    }

    private static float approximateVolume(CollisionShape shape) {
        return switch (shape.shapeType()) {
            case SPHERE -> {
                SphereCollisionShape s = (SphereCollisionShape) shape;
                yield (float) ((4.0 / 3.0) * Math.PI * s.radius() * s.radius() * s.radius());
            }
            case BOX -> {
                BoxCollisionShape b = (BoxCollisionShape) shape;
                yield (b.halfExtentX() * 2f) * (b.halfExtentY() * 2f) * (b.halfExtentZ() * 2f);
            }
            case CAPSULE -> {
                CapsuleCollisionShape c = (CapsuleCollisionShape) shape;
                float cyl = (float) (Math.PI * c.radius() * c.radius() * c.height());
                float hemi = (float) ((4.0 / 3.0) * Math.PI * c.radius() * c.radius() * c.radius());
                yield cyl + hemi;
            }
            case CYLINDER -> {
                CylinderCollisionShape c = (CylinderCollisionShape) shape;
                yield (float) (Math.PI * c.radius() * c.radius() * c.height());
            }
            default -> {
                var aabb = shape.getWorldBounds(new Transformf().identity());
                yield (float) (aabb.sizeX() * aabb.sizeY() * aabb.sizeZ());
            }
        };
    }
}
