package org.dynamisphysics.ode4j.world;

import org.dynamisphysics.ode4j.body.Ode4jForceAccumulator;
import org.dynamisphysics.ode4j.character.Ode4jCharacterController;
import org.dynamisphysics.ode4j.constraint.Ode4jConstraintRegistry;
import org.dynamisphysics.ode4j.event.Ode4jContactDispatcher;
import org.dynamisphysics.ode4j.vehicle.Ode4jVehicleSystem;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.math.DVector3;

public final class Ode4jStepLoop {
    private final DWorld world;
    private final DHashSpace space;
    private final DJointGroup contactGroup;
    private final Ode4jForceAccumulator forceAccumulator;
    private final Ode4jContactDispatcher dispatcher;
    private final Ode4jConstraintRegistry constraintRegistry;
    private final Ode4jVehicleSystem vehicleSystem;
    private final Ode4jCharacterController characterController;

    private int stepCount = 0;
    private float lastStepMs = 0f;

    public Ode4jStepLoop(
        DWorld world,
        DHashSpace space,
        DJointGroup contactGroup,
        Ode4jForceAccumulator forceAccumulator,
        Ode4jContactDispatcher dispatcher,
        Ode4jConstraintRegistry constraintRegistry,
        Ode4jVehicleSystem vehicleSystem,
        Ode4jCharacterController characterController
    ) {
        this.world = world;
        this.space = space;
        this.contactGroup = contactGroup;
        this.forceAccumulator = forceAccumulator;
        this.dispatcher = dispatcher;
        this.constraintRegistry = constraintRegistry;
        this.vehicleSystem = vehicleSystem;
        this.characterController = characterController;
    }

    public void step(float deltaSeconds, int subSteps) {
        float dt = deltaSeconds / subSteps;
        long start = System.nanoTime();
        DVector3 gravity = new DVector3();
        world.getGravity(gravity);

        for (int i = 0; i < subSteps; i++) {
            // Apply queued external forces/impulses for this substep.
            forceAccumulator.flush();

            // Vehicle and character controllers must contribute forces before integration.
            vehicleSystem.stepAll(dt);
            characterController.stepAll(
                dt,
                new org.vectrix.core.Vector3f(
                    (float) gravity.get0(),
                    (float) gravity.get1(),
                    (float) gravity.get2()
                )
            );

            // Build contact joints and solve in the same integration step.
            OdeHelper.spaceCollide(space, null, dispatcher.callback);
            world.quickStep(dt);

            constraintRegistry.checkBreakForces();
            contactGroup.empty();
        }

        lastStepMs = (System.nanoTime() - start) / 1_000_000f;
        stepCount++;
    }

    public int stepCount() { return stepCount; }
    public float lastStepMs() { return lastStepMs; }
    public DWorld world() { return world; }
    public DHashSpace space() { return space; }
}
