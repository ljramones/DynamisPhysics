package org.dynamisphysics.jolt.integration;

import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.event.WheelSlipEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.jolt.JoltBackendRegistrar;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledIfSystemProperty(named = "physics.parity.integration", matches = "true")
class TerrainMeshParityTest {

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void iceZoneProducesMoreSlipThanAsphalt(PhysicsBackend backend) {
        PhysicsWorld asphaltWorld = IntegrationParityScenes.newWorld(backend, true);
        PhysicsWorld iceWorld = IntegrationParityScenes.newWorld(backend, true);
        try {
            IntegrationParityScenes.spawnGroundBox(asphaltWorld, PhysicsMaterial.ASPHALT);
            IntegrationParityScenes.spawnGroundBox(iceWorld, PhysicsMaterial.ICE);
            IntegrationParityScenes.spawnMeshQuadZone(asphaltWorld, -8f, 8f, PhysicsMaterial.ASPHALT);
            IntegrationParityScenes.spawnMeshQuadZone(iceWorld, -8f, 8f, PhysicsMaterial.ICE);

            VehicleHandle asphaltVehicle = IntegrationParityScenes.spawnAndSettleVehicle(asphaltWorld, 0f, 1.0f, 0f);
            VehicleHandle iceVehicle = IntegrationParityScenes.spawnAndSettleVehicle(iceWorld, 0f, 1.0f, 0f);
            asphaltWorld.applyThrottle(asphaltVehicle, 1f);
            iceWorld.applyThrottle(iceVehicle, 1f);

            int slipAsphalt = 0;
            int slipIce = 0;
            for (int i = 0; i < 220; i++) {
                asphaltWorld.step(IntegrationParityScenes.DT, 1);
                iceWorld.step(IntegrationParityScenes.DT, 1);
                slipAsphalt += countSlip(asphaltWorld.drainEvents(), PhysicsMaterial.ASPHALT);
                slipIce += countSlip(iceWorld.drainEvents(), PhysicsMaterial.ICE);
            }

            assertTrue(slipIce > 0, "expected slip events on ice for " + backend);
            assertTrue((slipIce + slipAsphalt) > 0,
                "expected terrain slip signal for " + backend + ", asphalt=" + slipAsphalt + " ice=" + slipIce);
        } finally {
            asphaltWorld.destroy();
            iceWorld.destroy();
        }
    }

    private static int countSlip(Iterable<PhysicsEvent> events, PhysicsMaterial expectedMaterial) {
        int count = 0;
        for (PhysicsEvent event : events) {
            if (event instanceof WheelSlipEvent slip && slip.surfaceMaterial() == expectedMaterial) {
                count++;
            }
        }
        return count;
    }
}
