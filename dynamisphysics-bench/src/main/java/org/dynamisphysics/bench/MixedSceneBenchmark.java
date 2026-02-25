package org.dynamisphysics.bench;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Measurement;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Param;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.annotations.TearDown;
import org.openjdk.jmh.annotations.Warmup;
import org.openjdk.jmh.infra.Blackhole;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@BenchmarkMode(Mode.Throughput)
@OutputTimeUnit(TimeUnit.SECONDS)
@Warmup(iterations = 2, time = 1)
@Measurement(iterations = 4, time = 1)
@Fork(1)
public class MixedSceneBenchmark {

    @State(Scope.Thread)
    public static class BenchState {
        @Param({"ODE4J", "JOLT"})
        public String backend;

        @Param({"1", "10"})
        public int vehicleCount;

        @Param({"1", "10"})
        public int ragdollCount;

        @Param({"10", "100"})
        public int constraintModules;

        PhysicsWorld world;
        List<VehicleHandle> vehicles;
        List<RagdollHandle> ragdolls;
        List<RigidBodyHandle> compounds;
        int steps;

        @Setup(Level.Trial)
        public void setup() {
            PhysicsBackend selected = PhysicsBackend.valueOf(backend);
            world = BenchSupport.createWorld(selected, false);
            BenchSupport.spawnMeshGround(world);
            vehicles = spawnVehicles(world, vehicleCount);
            ragdolls = spawnRagdolls(world, ragdollCount);
            compounds = BenchSupport.spawnCompoundPile(world, 40, 4);
            addMechanicalModules(world, constraintModules);
            BenchSupport.warmStart(world, 20);
            steps = 0;
        }

        @TearDown(Level.Trial)
        public void teardown() {
            if (world != null) {
                world.destroy();
            }
        }

        private static List<VehicleHandle> spawnVehicles(PhysicsWorld world, int count) {
            List<VehicleHandle> handles = new ArrayList<>(count);
            for (int i = 0; i < count; i++) {
                VehicleHandle handle = world.spawnVehicle(BenchSupport.defaultVehicleDescriptor());
                world.teleport(handle.chassisBody(), new Vector3f((i % 5) * 4f, 1.5f, (i / 5) * 6f), new Quaternionf());
                world.applyThrottle(handle, 0.6f);
                handles.add(handle);
            }
            return handles;
        }

        private static List<RagdollHandle> spawnRagdolls(PhysicsWorld world, int count) {
            List<RagdollHandle> handles = new ArrayList<>(count);
            for (int i = 0; i < count; i++) {
                RagdollHandle handle = world.spawnRagdoll(BenchSupport.simpleRagdollDescriptor(), BenchSupport.simpleRagdollPose(6f + i * 0.5f));
                world.activateRagdoll(handle, 0f);
                handles.add(handle);
            }
            return handles;
        }

        private static void addMechanicalModules(PhysicsWorld world, int count) {
            List<RigidBodyHandle> moduleBodies = BenchSupport.spawnConstraintBodies(world, count + 1);
            for (int i = 0; i < count; i++) {
                BenchSupport.addMechanicalModule(world, moduleBodies.get(i), moduleBodies.get(i + 1), i);
            }
        }
    }

    @Benchmark
    public void stepMixedScene(BenchState state, Blackhole bh) {
        state.world.step(1f / 60f, 1);
        state.steps++;
        if ((state.steps % 10) == 0) {
            bh.consume(state.world.drainEvents().size());
        }
        VehicleState v = state.world.getVehicleState(state.vehicles.get(0));
        bh.consume(v.speed());
        bh.consume(state.ragdolls.get(0).getBoneState("root").position().y());
        bh.consume(state.world.getBodyState(state.compounds.get(0)).position().y());
    }
}
