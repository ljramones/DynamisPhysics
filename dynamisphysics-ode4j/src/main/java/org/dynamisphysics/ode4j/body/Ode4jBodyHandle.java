package org.dynamisphysics.ode4j.body;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.body.StableRigidBodyId;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;

import java.util.ArrayList;
import java.util.List;

public final class Ode4jBodyHandle implements RigidBodyHandle, StableRigidBodyId {
    public record GeomRef(Ode4jBodyHandle handle, int geomId) {}

    private final int bodyId;
    private final int geomId;
    private final DBody body;
    private final DGeom geom;
    private final List<DGeom> ownedGeoms;
    private final RigidBodyConfig config;
    private boolean alive = true;

    public Ode4jBodyHandle(int bodyId, int geomId, DBody body, DGeom geom, RigidBodyConfig config) {
        this(bodyId, geomId, body, geom, List.of(geom), config);
    }

    public Ode4jBodyHandle(
        int bodyId,
        int geomId,
        DBody body,
        DGeom geom,
        List<DGeom> ownedGeoms,
        RigidBodyConfig config
    ) {
        this.bodyId = bodyId;
        this.geomId = geomId;
        this.body = body;
        this.geom = geom;
        this.ownedGeoms = new ArrayList<>(ownedGeoms);
        this.config = config;
        if (body != null) {
            body.setData(this);
        }
        for (int i = 0; i < this.ownedGeoms.size(); i++) {
            int id = geomId + i;
            this.ownedGeoms.get(i).setData(new GeomRef(this, id));
        }
    }

    public DBody body() { return body; }
    public DGeom geom() { return geom; }
    public RigidBodyConfig config() { return config; }
    public int bodyId() { return bodyId; }
    public int geomId() { return geomId; }
    public List<DGeom> ownedGeoms() { return ownedGeoms; }

    public void kill() {
        alive = false;
        for (DGeom g : ownedGeoms) {
            g.destroy();
        }
        if (body != null) {
            body.destroy();
        }
    }

    @Override public boolean isAlive() { return alive; }
    @Override public int layer() { return config.layer(); }
    @Override public Object userData() { return config.userData(); }
    @Override public BodyMode mode() { return config.mode(); }
}
