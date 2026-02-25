module org.dynamisphysics.jolt {
    requires org.dynamisphysics.api;
    requires org.animis;
    requires org.vectrix;
    requires org.dynamiscollision;
    requires meshforge;
    requires dynamis.gpu.api;
    requires jolt.jni;

    exports org.dynamisphysics.jolt;
    exports org.dynamisphysics.jolt.world;
}
