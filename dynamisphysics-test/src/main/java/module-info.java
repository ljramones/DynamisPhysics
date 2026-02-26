module org.dynamisphysics.test {
    requires org.dynamisphysics.api;
    requires org.vectrix;
    requires org.dynamiscollision;
    requires org.animis;
    requires org.junit.jupiter.api;
    requires com.fasterxml.jackson.databind;

    exports org.dynamisphysics.test.mock;
    exports org.dynamisphysics.test.harness;
    exports org.dynamisphysics.test.assertions;
    exports org.dynamisphysics.test.scene;
    exports org.dynamisphysics.test.replay;
}
