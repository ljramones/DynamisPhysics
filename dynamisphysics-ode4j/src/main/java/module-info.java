module org.dynamisengine.physics.ode4j {
    requires org.dynamisengine.physics.api;
    requires org.dynamisengine.vectrix;
    requires org.dynamisengine.animis;
    requires org.dynamisengine.collision;
    requires org.dynamisengine.gpu.api;
    requires core; // ode4j 0.5.x has no module name: its automatic name comes from core-<version>.jar
    requires org.dynamisengine.meshforge;

    exports org.dynamisengine.physics.ode4j;
    exports org.dynamisengine.physics.ode4j.debug;
    exports org.dynamisengine.physics.ode4j.world;
}
