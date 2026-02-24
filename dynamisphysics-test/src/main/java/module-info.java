module org.dynamisphysics.test {
    requires org.dynamisphysics.api;
    requires collision.detection;
    requires org.vectrix;
    requires org.junit.jupiter.api;

    exports org.dynamisphysics.test;
    exports org.dynamisphysics.test.mock;
    exports org.dynamisphysics.test.harness;
    exports org.dynamisphysics.test.assertions;
    exports org.dynamisphysics.test.scene;
}
