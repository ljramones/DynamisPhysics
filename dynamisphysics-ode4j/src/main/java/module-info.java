module org.dynamisphysics.ode4j {
    requires org.dynamisphysics.api;
    requires org.animis;
    requires org.vectrix;
    requires collision.detection;
    requires meshforge;
    requires dynamis.gpu.api;
    requires core;

    exports org.dynamisphysics.ode4j;
    exports org.dynamisphysics.ode4j.world;
}
