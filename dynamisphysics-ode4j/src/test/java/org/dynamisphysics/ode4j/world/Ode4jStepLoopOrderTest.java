package org.dynamisphysics.ode4j.world;

import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.dynamisphysics.ode4j.body.Ode4jForceAccumulator;
import org.dynamisphysics.ode4j.character.Ode4jCharacterController;
import org.dynamisphysics.ode4j.constraint.Ode4jConstraintRegistry;
import org.dynamisphysics.ode4j.event.Ode4jContactDispatcher;
import org.dynamisphysics.ode4j.event.Ode4jEventBuffer;
import org.dynamisphysics.ode4j.query.Ode4jRaycastExecutor;
import org.dynamisphysics.ode4j.ragdoll.Ode4jRagdollSystem;
import org.dynamisphysics.ode4j.vehicle.Ode4jVehicleSystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class Ode4jStepLoopOrderTest {
    private DWorld world;
    private DHashSpace space;
    private DJointGroup contactGroup;

    @BeforeEach
    void setUp() {
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createHashSpace(null);
        contactGroup = OdeHelper.createJointGroup();
        world.setGravity(0.0, -9.81, 0.0);
    }

    @AfterEach
    void tearDown() {
        contactGroup.destroy();
        space.destroy();
        world.destroy();
        OdeHelper.closeODE();
    }

    @Test
    void stepLoopPhaseOrderIsStable() {
        Ode4jEventBuffer eventBuffer = new Ode4jEventBuffer();
        Ode4jForceAccumulator forceAccumulator = new Ode4jForceAccumulator();
        Ode4jBodyRegistry bodyRegistry = new Ode4jBodyRegistry(world, space);
        Ode4jConstraintRegistry constraintRegistry = new Ode4jConstraintRegistry(world, bodyRegistry, eventBuffer);
        Ode4jRaycastExecutor raycastExecutor = new Ode4jRaycastExecutor(space);
        Ode4jVehicleSystem vehicleSystem = new Ode4jVehicleSystem(bodyRegistry, eventBuffer, raycastExecutor);
        Ode4jCharacterController characterController = new Ode4jCharacterController(bodyRegistry, raycastExecutor, eventBuffer);
        Ode4jRagdollSystem ragdollSystem = new Ode4jRagdollSystem(bodyRegistry, constraintRegistry);
        Ode4jContactDispatcher dispatcher = new Ode4jContactDispatcher(world, contactGroup, eventBuffer);

        List<String> phases = new ArrayList<>();
        Ode4jStepLoop stepLoop = new Ode4jStepLoop(
            world,
            space,
            contactGroup,
            forceAccumulator,
            dispatcher,
            constraintRegistry,
            vehicleSystem,
            characterController,
            ragdollSystem,
            phases::add
        );

        stepLoop.step(1f / 60f, 1);

        assertEquals(List.of(
            "forceAccumulator.flush",
            "vehicleSystem.stepAll",
            "characterController.stepAll",
            "spaceCollide",
            "quickStep",
            "constraintRegistry.checkBreakForces",
            "contactGroup.empty",
            "ragdollSystem.stepAll"
        ), phases);
    }
}
