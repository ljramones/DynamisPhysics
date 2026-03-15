module org.dynamisengine.physics.test {
    requires org.dynamisengine.physics.api;
    requires org.dynamisengine.vectrix;
    requires org.dynamisengine.collision;
    requires org.animis;
    requires org.junit.jupiter.api;
    requires com.fasterxml.jackson.databind;

    exports org.dynamisengine.physics.test.mock;
    exports org.dynamisengine.physics.test.harness;
    exports org.dynamisengine.physics.test.assertions;
    exports org.dynamisengine.physics.test.scene;
    exports org.dynamisengine.physics.test.replay;
}
