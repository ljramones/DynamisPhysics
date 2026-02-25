package org.dynamisphysics.bench;

import org.dynamisphysics.api.VehicleHandle;
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
public class VehicleBenchmark {

    @State(Scope.Thread)
    public static class BenchState {
        @Param({"ODE4J", "JOLT"})
        public String backend;

        @Param({"10", "100"})
        public int vehicleCount;

        PhysicsWorld world;
        List<VehicleHandle> vehicles;

        @Setup(Level.Trial)
        public void setup() {
            PhysicsBackend selected = PhysicsBackend.valueOf(backend);
            world = BenchSupport.createWorld(selected, false);
            BenchSupport.spawnGround(world);
            vehicles = new ArrayList<>(vehicleCount);
            for (int i = 0; i < vehicleCount; i++) {
                VehicleHandle handle = world.spawnVehicle(BenchSupport.defaultVehicleDescriptor());
                world.teleport(handle.chassisBody(), new Vector3f((i % 20) * 4f, 1.5f, (i / 20) * 6f), new Quaternionf());
                world.applyThrottle(handle, 0.65f);
                vehicles.add(handle);
            }
            BenchSupport.warmStart(world, 20);
        }

        @TearDown(Level.Trial)
        public void teardown() {
            if (world != null) {
                world.destroy();
            }
        }
    }

    @Benchmark
    public void stepVehicleFleet(BenchState state, Blackhole bh) {
        state.world.step(1f / 60f, 1);
        if (!state.vehicles.isEmpty()) {
            VehicleState sample = state.world.getVehicleState(state.vehicles.get(0));
            bh.consume(sample.speed());
            bh.consume(sample.engineRpm());
        }
    }
}
