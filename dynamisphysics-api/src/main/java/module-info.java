module org.dynamisengine.physics.api {
    requires org.dynamisengine.collision;
    requires org.dynamisengine.animis;
    requires org.dynamisengine.vectrix;

    exports org.dynamisengine.physics.api;
    exports org.dynamisengine.physics.api.world;
    exports org.dynamisengine.physics.api.body;
    exports org.dynamisengine.physics.api.shape;
    exports org.dynamisengine.physics.api.material;
    exports org.dynamisengine.physics.api.constraint;
    exports org.dynamisengine.physics.api.event;
    exports org.dynamisengine.physics.api.query;
    exports org.dynamisengine.physics.api.config;
    exports org.dynamisengine.physics.api.collision;
}
