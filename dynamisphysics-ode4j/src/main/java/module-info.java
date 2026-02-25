module org.dynamisphysics.ode4j {
    requires org.dynamisphysics.api;
    requires org.vectrix;
    requires org.animis;
    requires org.dynamiscollision;
    requires dynamis.gpu.api;
    requires org.ode4j;
    requires meshforge;

    exports org.dynamisphysics.ode4j;
    exports org.dynamisphysics.ode4j.debug;
    exports org.dynamisphysics.ode4j.world;
}
