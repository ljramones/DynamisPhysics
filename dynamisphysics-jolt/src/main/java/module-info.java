module org.dynamisphysics.jolt {
    requires org.dynamisphysics.api;
    requires org.animis;
    requires org.vectrix;
    requires org.dynamiscollision;
    requires com.github.stephengold.joltjni;
    requires meshforge;

    exports org.dynamisphysics.jolt;
    exports org.dynamisphysics.jolt.world;
}
