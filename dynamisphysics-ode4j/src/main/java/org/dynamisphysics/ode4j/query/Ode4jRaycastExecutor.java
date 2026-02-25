package org.dynamisphysics.ode4j.query;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DRay;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public final class Ode4jRaycastExecutor {
    private final DSpace space;

    public Ode4jRaycastExecutor(DSpace space) {
        this.space = space;
    }

    public Optional<RaycastResult> raycastClosest(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        DRay ray = OdeHelper.createRay((DSpace) null, maxDist);
        ray.set(origin.x(), origin.y(), origin.z(), dir.x(), dir.y(), dir.z());

        AtomicReference<RaycastResult> closest = new AtomicReference<>();

        OdeHelper.spaceCollide2(ray, space, null, (data, o1, o2) -> {
            DGeom target = o1 == ray ? o2 : o1;
            DContactGeomBuffer buf = new DContactGeomBuffer(1);
            int n = OdeHelper.collide(ray, target, 1, buf);
            if (n == 0) {
                return;
            }
            DContactGeom cg = buf.get(0);
            float dx = (float) cg.pos.get0() - origin.x();
            float dy = (float) cg.pos.get1() - origin.y();
            float dz = (float) cg.pos.get2() - origin.z();
            float fraction = (float) (Math.sqrt(dx * dx + dy * dy + dz * dz) / maxDist);
            PhysicsMaterial mat = PhysicsMaterial.DEFAULT;
            RigidBodyHandle handle = null;
            Object data2 = target.getData();
            if (data2 instanceof Ode4jBodyHandle h) {
                mat = h.config().material();
                handle = h;
            }
            RaycastResult result = new RaycastResult(
                handle,
                new Vector3f((float) cg.pos.get0(), (float) cg.pos.get1(), (float) cg.pos.get2()),
                new Vector3f((float) cg.normal.get0(), (float) cg.normal.get1(), (float) cg.normal.get2()),
                fraction,
                mat,
                0
            );
            RaycastResult prev = closest.get();
            if (prev == null || fraction < prev.fraction()) {
                closest.set(result);
            }
        });

        ray.destroy();
        return Optional.ofNullable(closest.get());
    }

    public List<RaycastResult> raycastAll(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        DRay ray = OdeHelper.createRay((DSpace) null, maxDist);
        ray.set(origin.x(), origin.y(), origin.z(), dir.x(), dir.y(), dir.z());
        List<RaycastResult> hits = new ArrayList<>();

        OdeHelper.spaceCollide2(ray, space, null, (data, o1, o2) -> {
            DGeom target = o1 == ray ? o2 : o1;
            DContactGeomBuffer buf = new DContactGeomBuffer(1);
            int n = OdeHelper.collide(ray, target, 1, buf);
            if (n == 0) {
                return;
            }
            DContactGeom cg = buf.get(0);
            float dx = (float) cg.pos.get0() - origin.x();
            float dy = (float) cg.pos.get1() - origin.y();
            float dz = (float) cg.pos.get2() - origin.z();
            float fraction = (float) (Math.sqrt(dx * dx + dy * dy + dz * dz) / maxDist);
            PhysicsMaterial mat = PhysicsMaterial.DEFAULT;
            RigidBodyHandle handle = null;
            Object data2 = target.getData();
            if (data2 instanceof Ode4jBodyHandle h) {
                mat = h.config().material();
                handle = h;
            }
            hits.add(new RaycastResult(
                handle,
                new Vector3f((float) cg.pos.get0(), (float) cg.pos.get1(), (float) cg.pos.get2()),
                new Vector3f((float) cg.normal.get0(), (float) cg.normal.get1(), (float) cg.normal.get2()),
                fraction,
                mat,
                0
            ));
        });

        ray.destroy();
        hits.sort((a, b) -> Float.compare(a.fraction(), b.fraction()));
        return hits;
    }
}
