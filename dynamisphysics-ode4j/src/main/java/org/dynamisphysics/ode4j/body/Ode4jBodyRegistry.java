package org.dynamisphysics.ode4j.body;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.ode4j.shape.Ode4jShapeAdapter;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toOde;

public final class Ode4jBodyRegistry {
    private final DWorld world;
    private final DSpace space;
    private final Map<RigidBodyHandle, Ode4jBodyHandle> handlesByHandle = new LinkedHashMap<>();
    private final Map<Integer, Ode4jBodyHandle> handlesById = new LinkedHashMap<>();
    private final Map<Integer, Ode4jBodyHandle> lookupById = new HashMap<>();
    private int nextBodyId = 1;
    private int nextGeomId = 1;

    public Ode4jBodyRegistry(DWorld world, DSpace space) {
        this.world = world;
        this.space = space;
    }

    public Ode4jBodyHandle spawn(RigidBodyConfig config) {
        return spawnWithIds(config, nextBodyId++, nextGeomId++);
    }

    public Ode4jBodyHandle spawnWithIds(RigidBodyConfig config, int bodyId, int geomId) {
        nextBodyId = Math.max(nextBodyId, bodyId + 1);
        nextGeomId = Math.max(nextGeomId, geomId + 1);
        DGeom geom = Ode4jShapeAdapter.toGeom(config.shape(), space);
        DBody body = null;

        if (config.mode() == BodyMode.DYNAMIC || config.mode() == BodyMode.KINEMATIC) {
            body = OdeHelper.createBody(world);
            Ode4jInertiaComputer.applyInertia(body, config.shape(), config.mass());
        }

        applyWorldTransform(body, geom, config.worldTransform());

        if (config.mode() == BodyMode.KINEMATIC && body != null) {
            body.setKinematic();
        }

        if (config.mode() == BodyMode.STATIC || body == null) {
            geom.setBody(null);
        } else {
            geom.setBody(body);
        }

        if (body != null && config.linearVelocity() != null) {
            body.setLinearVel(toOde(config.linearVelocity()));
        }
        if (body != null && config.angularVelocity() != null) {
            body.setAngularVel(toOde(config.angularVelocity()));
        }
        if (body != null) {
            body.setGravityMode(config.gravityScale() > 0f);
        }

        var handle = new Ode4jBodyHandle(bodyId, geomId, body, geom, config);
        handlesByHandle.put(handle, handle);
        handlesById.put(handle.bodyId(), handle);
        lookupById.put(handle.bodyId(), handle);
        return handle;
    }

    public void destroy(RigidBodyHandle h) {
        Ode4jBodyHandle oh = handlesByHandle.remove(h);
        if (oh != null) {
            handlesById.remove(oh.bodyId());
            lookupById.remove(oh.bodyId());
            oh.kill();
        }
    }

    public BodyState getState(RigidBodyHandle h) {
        Ode4jBodyHandle oh = handlesByHandle.get(h);
        if (oh == null || !oh.isAlive()) {
            return BodyState.ZERO;
        }
        return readState(oh);
    }

    public void setState(RigidBodyHandle h, BodyState s) {
        Ode4jBodyHandle oh = handlesByHandle.get(h);
        if (oh == null || oh.body() == null) {
            return;
        }
        oh.body().setPosition(toOde(s.position()));
        oh.body().setQuaternion(toOde(s.orientation()));
        oh.body().setLinearVel(toOde(s.linearVelocity()));
        oh.body().setAngularVel(toOde(s.angularVelocity()));
    }

    public Collection<Ode4jBodyHandle> allHandles() {
        return Collections.unmodifiableCollection(handlesByHandle.values());
    }

    public int bodyCount() {
        return handlesByHandle.size();
    }

    public Ode4jBodyHandle getHandle(RigidBodyHandle h) {
        return handlesByHandle.get(h);
    }

    public Ode4jBodyHandle getHandleById(int id) {
        return lookupById.get(id);
    }

    public List<Ode4jBodyHandle> bodiesInIdOrder() {
        return handlesById.values().stream()
            .sorted(Comparator.comparingInt(Ode4jBodyHandle::bodyId))
            .toList();
    }

    public void clearAllBodies() {
        var snapshot = List.copyOf(handlesByHandle.keySet());
        for (RigidBodyHandle h : snapshot) {
            destroy(h);
        }
    }

    public int nextBodyId() {
        return nextBodyId;
    }

    public int nextGeomId() {
        return nextGeomId;
    }

    private static BodyState readState(Ode4jBodyHandle h) {
        if (h.body() == null) {
            DVector3C pos = h.geom().getPosition();
            return new BodyState(new Vector3f((float) pos.get0(), (float) pos.get1(), (float) pos.get2()),
                new Quaternionf(), new Vector3f(), new Vector3f(), true);
        }
        DVector3C lv = h.body().getLinearVel();
        DVector3C av = h.body().getAngularVel();
        DVector3C pos = h.body().getPosition();
        DQuaternionC q = h.body().getQuaternion();
        boolean sleeping = !h.body().isEnabled();
        return new BodyState(
            new Vector3f((float) pos.get0(), (float) pos.get1(), (float) pos.get2()),
            new Quaternionf((float) q.get1(), (float) q.get2(), (float) q.get3(), (float) q.get0()),
            new Vector3f((float) lv.get0(), (float) lv.get1(), (float) lv.get2()),
            new Vector3f((float) av.get0(), (float) av.get1(), (float) av.get2()),
            sleeping
        );
    }

    private static void applyWorldTransform(DBody body, DGeom geom, Matrix4f m) {
        Vector3f pos = new Vector3f();
        m.getTranslation(pos);
        Quaternionf ori = new Quaternionf();
        m.getUnnormalizedRotation(ori);
        if (body != null) {
            body.setPosition(toOde(pos));
            body.setQuaternion(toOde(ori));
            return;
        }
        // Plane and space geoms are not placeable in ODE; skip explicit transforms.
        if (geom.getClassID() == DGeom.dPlaneClass || geom.isSpace()) {
            return;
        }
        geom.setPosition(toOde(pos));
        geom.setQuaternion(toOde(ori));
    }
}
