package org.dynamisphysics.api.body;

public interface RigidBodyHandle {
    boolean isAlive();
    int layer();
    Object userData();
    BodyMode mode();
}
