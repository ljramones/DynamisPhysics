package org.dynamisphysics.ode4j.body;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;

public final class Ode4jBodyHandle implements RigidBodyHandle {
    private final int bodyId;
    private final int geomId;
    private final DBody body;
    private final DGeom geom;
    private final RigidBodyConfig config;
    private boolean alive = true;

    public Ode4jBodyHandle(int bodyId, int geomId, DBody body, DGeom geom, RigidBodyConfig config) {
        this.bodyId = bodyId;
        this.geomId = geomId;
        this.body = body;
        this.geom = geom;
        this.config = config;
        if (body != null) {
            body.setData(this);
        }
        geom.setData(this);
    }

    public DBody body() { return body; }
    public DGeom geom() { return geom; }
    public RigidBodyConfig config() { return config; }
    public int bodyId() { return bodyId; }
    public int geomId() { return geomId; }

    public void kill() {
        alive = false;
        if (body != null) {
            body.destroy();
        }
        geom.destroy();
    }

    @Override public boolean isAlive() { return alive; }
    @Override public int layer() { return config.layer(); }
    @Override public Object userData() { return config.userData(); }
    @Override public BodyMode mode() { return config.mode(); }
}
