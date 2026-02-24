module org.dynamisphysics.jolt {
    requires org.dynamisphysics.api;
    requires org.vectrix;
    requires org.jolt.jni;     // adjust to match your Jolt module name

    exports org.dynamisphysics.jolt;
    exports org.dynamisphysics.jolt.world;
    exports org.dynamisphysics.jolt.body;
    exports org.dynamisphysics.jolt.shape;
    exports org.dynamisphysics.jolt.constraint;
    exports org.dynamisphysics.jolt.event;
    exports org.dynamisphysics.jolt.query;
    exports org.dynamisphysics.jolt.debug;
}
