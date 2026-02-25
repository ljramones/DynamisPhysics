module org.dynamisphysics.ode4j {
    requires org.dynamisphysics.api;
    requires org.vectrix;
    requires org.animis;
    requires collision.detection;
    requires meshforge;
    requires dynamis.gpu.api;
    requires org.ode4j;

    exports org.dynamisphysics.ode4j;
    exports org.dynamisphysics.ode4j.world;
}
