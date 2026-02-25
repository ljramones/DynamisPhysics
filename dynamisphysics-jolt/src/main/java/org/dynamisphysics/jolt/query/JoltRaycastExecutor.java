package org.dynamisphysics.jolt.query;

import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.NarrowPhaseQuery;
import com.github.stephengold.joltjni.RRayCast;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.RayCastResult;
import com.github.stephengold.joltjni.Vec3;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.vectrix.core.Vector3f;

import java.util.List;
import java.util.Optional;

import static org.dynamisphysics.jolt.world.JoltConversions.toVector3f;

public final class JoltRaycastExecutor {
    private final NarrowPhaseQuery query;
    private final BodyInterface bodyInterface;
    private final JoltBodyRegistry bodyRegistry;

    public JoltRaycastExecutor(NarrowPhaseQuery query, BodyInterface bodyInterface, JoltBodyRegistry bodyRegistry) {
        this.query = query;
        this.bodyInterface = bodyInterface;
        this.bodyRegistry = bodyRegistry;
    }

    public Optional<RaycastResult> raycastClosest(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        Vec3 direction = new Vec3(dir.x(), dir.y(), dir.z());
        if (direction.lengthSq() < 1e-12f) {
            return Optional.empty();
        }
        direction.normalizeInPlace();
        direction.scaleInPlace(maxDist);

        RRayCast ray = new RRayCast(new RVec3(origin.x(), origin.y(), origin.z()), direction);
        RayCastResult hit = new RayCastResult();
        if (!query.castRay(ray, hit)) {
            return Optional.empty();
        }

        float fraction = hit.getFraction();
        Vector3f point = toVector3f(ray.getPointOnRay(fraction));
        int joltBodyId = hit.getBodyId();
        JoltBodyHandle handle = bodyRegistry.getByJoltId(joltBodyId);
        Vector3f normal = new Vector3f(0f, 1f, 0f);

        return Optional.of(new RaycastResult(
            handle,
            point,
            normal,
            fraction,
            handle != null ? handle.config().material() : org.dynamisphysics.api.material.PhysicsMaterial.DEFAULT,
            handle != null ? handle.layer() : 0
        ));
    }

    public List<RaycastResult> raycastAll(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        return raycastClosest(origin, dir, maxDist, layerMask).stream().toList();
    }
}
