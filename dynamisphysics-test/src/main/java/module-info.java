module org.dynamisphysics.test {
    requires org.dynamisphysics.api;
    requires org.vectrix;
    requires collision.detection;
    requires org.animis;
    requires org.junit.jupiter.api;

    exports org.dynamisphysics.test.mock;
    exports org.dynamisphysics.test.harness;
    exports org.dynamisphysics.test.assertions;
    exports org.dynamisphysics.test.scene;
}
