module org.dynamisphysics.ode4j {
    requires org.dynamisphysics.api;
    requires org.vectrix;
    requires org.ode4j.core;     // adjust to match your ODE4J module name

    exports org.dynamisphysics.ode4j;
    exports org.dynamisphysics.ode4j.world;
    exports org.dynamisphysics.ode4j.body;
    exports org.dynamisphysics.ode4j.shape;
    exports org.dynamisphysics.ode4j.constraint;
    exports org.dynamisphysics.ode4j.event;
    exports org.dynamisphysics.ode4j.query;
    exports org.dynamisphysics.ode4j.debug;
}
