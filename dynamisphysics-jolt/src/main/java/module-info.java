module org.dynamisengine.physics.jolt {
    requires org.dynamisengine.physics.api;
    requires org.animis;
    requires org.dynamisengine.vectrix;
    requires org.dynamisengine.collision;
    requires com.github.stephengold.joltjni;
    requires meshforge;

    exports org.dynamisengine.physics.jolt;
    exports org.dynamisengine.physics.jolt.world;
}
