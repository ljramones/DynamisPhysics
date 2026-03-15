module org.dynamisengine.physics.ode4j {
    requires org.dynamisengine.physics.api;
    requires org.dynamisengine.vectrix;
    requires org.dynamisengine.animis;
    requires org.dynamisengine.collision;
    requires dynamis.gpu.api;
    requires org.ode4j;
    requires meshforge;

    exports org.dynamisengine.physics.ode4j;
    exports org.dynamisengine.physics.ode4j.debug;
    exports org.dynamisengine.physics.ode4j.world;
}
