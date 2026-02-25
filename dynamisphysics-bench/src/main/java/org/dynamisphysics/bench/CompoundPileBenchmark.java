package org.dynamisphysics.bench;

import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
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

import java.util.List;
import java.util.concurrent.TimeUnit;

@BenchmarkMode(Mode.Throughput)
@OutputTimeUnit(TimeUnit.SECONDS)
@Warmup(iterations = 2, time = 1)
@Measurement(iterations = 4, time = 1)
@Fork(1)
public class CompoundPileBenchmark {

    @State(Scope.Thread)
    public static class BenchState {
        @Param({"ODE4J", "JOLT"})
        public String backend;

        @Param({"100", "1000"})
        public int compoundCount;

        @Param({"2", "4", "8"})
        public int childrenPerCompound;

        PhysicsWorld world;
        List<RigidBodyHandle> compounds;

        @Setup(Level.Trial)
        public void setup() {
            PhysicsBackend selected = PhysicsBackend.valueOf(backend);
            world = BenchSupport.createWorld(selected, false);
            BenchSupport.spawnMeshGround(world);
            compounds = BenchSupport.spawnCompoundPile(world, compoundCount, childrenPerCompound);
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
    public void stepCompoundPile(BenchState state, Blackhole bh) {
        state.world.step(1f / 60f, 1);
        BodyState sample = state.world.getBodyState(state.compounds.get(0));
        bh.consume(sample.position().y());
        bh.consume(sample.linearVelocity().length());
    }
}
