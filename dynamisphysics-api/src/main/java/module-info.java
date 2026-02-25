module org.dynamisphysics.api {
    requires org.dynamiscollision;
    requires org.animis;
    requires org.vectrix;

    exports org.dynamisphysics.api;
    exports org.dynamisphysics.api.world;
    exports org.dynamisphysics.api.body;
    exports org.dynamisphysics.api.shape;
    exports org.dynamisphysics.api.material;
    exports org.dynamisphysics.api.constraint;
    exports org.dynamisphysics.api.event;
    exports org.dynamisphysics.api.query;
    exports org.dynamisphysics.api.config;
}
