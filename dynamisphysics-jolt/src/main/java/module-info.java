module org.dynamisphysics.jolt {
    requires org.dynamisphysics.api;
    requires org.animis;
    requires org.dynamisengine.vectrix;
    requires org.dynamisengine.collision;
    requires com.github.stephengold.joltjni;
    requires meshforge;

    exports org.dynamisphysics.jolt;
    exports org.dynamisphysics.jolt.world;
}
