package org.dynamisphysics.bench;

import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.world.PhysicsWorld;
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
import org.vectrix.core.Vector3f;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

@BenchmarkMode(Mode.Throughput)
@OutputTimeUnit(TimeUnit.SECONDS)
@Warmup(iterations = 2, time = 1)
@Measurement(iterations = 4, time = 1)
@Fork(1)
public class RaycastBenchmark {

    @State(Scope.Thread)
    public static class BenchState {
        @Param({"ODE4J", "JOLT"})
        public String backend;

        @Param({"1", "100", "1000"})
        public int raysPerOp;

        PhysicsWorld world;
        Vector3f[] origins;
        Vector3f down = new Vector3f(0f, -1f, 0f);

        @Setup(Level.Trial)
        public void setup() {
            PhysicsBackend selected = PhysicsBackend.valueOf(backend);
            world = BenchSupport.createWorld(selected, true);
            BenchSupport.spawnGround(world);
            BenchSupport.spawnRaycastTargets(world, 2500);

            origins = new Vector3f[raysPerOp];
            int side = (int) java.lang.Math.ceil(java.lang.Math.sqrt(raysPerOp));
            for (int i = 0; i < raysPerOp; i++) {
                int x = i % side;
                int z = i / side;
                origins[i] = new Vector3f((x - side / 2f) * 2f, 50f, (z - side / 2f) * 2f);
            }

            BenchSupport.warmStart(world, 5);
        }

        @TearDown(Level.Trial)
        public void teardown() {
            if (world != null) {
                world.destroy();
            }
        }
    }

    @Benchmark
    public void raycastClosestBatch(BenchState state, Blackhole bh) {
        for (int i = 0; i < state.origins.length; i++) {
            Optional<RaycastResult> hit = state.world.raycastClosest(state.origins[i], state.down, 100f, -1);
            if (hit.isPresent()) {
                bh.consume(hit.get().fraction());
                bh.consume(hit.get().position().y());
            }
        }
    }
}
