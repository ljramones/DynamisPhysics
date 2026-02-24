package org.dynamisphysics.api;

/**
 * Root anchor for the DynamisPhysics API module.
 * All downstream consumers depend only on this module —
 * the ODE4J and Jolt implementations are engine-layer concerns.
 */
public final class DynamisPhysicsApi {
    public static final String VERSION = "0.1.0-SNAPSHOT";
    private DynamisPhysicsApi() {}
}
