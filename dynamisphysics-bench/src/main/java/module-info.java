module org.dynamisengine.physics.bench {
    requires org.dynamisengine.physics.api;
    requires org.dynamisengine.physics.ode4j;
    requires org.dynamisengine.physics.jolt;
    requires org.dynamisengine.physics.test;
    requires org.dynamisengine.vectrix;
    requires org.dynamisengine.collision;
    requires jmh.core;

    exports org.dynamisengine.physics.bench;
}
