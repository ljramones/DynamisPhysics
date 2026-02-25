package org.dynamisphysics.bench;

import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.world.PhysicsStats;
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
public class ConstraintSolveBenchmark {

    @State(Scope.Thread)
    public static class BenchState {
        @Param({"ODE4J", "JOLT"})
        public String backend;

        @Param({"100", "1000", "5000"})
        public int constraintCount;

        @Param({"SPRING_ONLY", "MECH_ONLY", "MIXED"})
        public String constraintTypeMix;

        PhysicsWorld world;
        List<RigidBodyHandle> bodies;

        @Setup(Level.Trial)
        public void setup() {
            PhysicsBackend selected = PhysicsBackend.valueOf(backend);
            world = BenchSupport.createWorld(selected, false);
            world.setGravity(new org.vectrix.core.Vector3f(0f, 0f, 0f));
            bodies = BenchSupport.spawnConstraintBodies(world, constraintCount + 1);

            for (int i = 0; i < constraintCount; i++) {
                RigidBodyHandle a = bodies.get(i);
                RigidBodyHandle b = bodies.get(i + 1);
                switch (constraintTypeMix) {
                    case "SPRING_ONLY" -> world.addConstraint(BenchSupport.springConstraint(a, b, 120f, 20f));
                    case "MECH_ONLY" -> BenchSupport.addMechanicalModule(world, a, b, i);
                    case "MIXED" -> {
                        if ((i & 1) == 0) {
                            world.addConstraint(BenchSupport.springConstraint(a, b, 120f, 20f));
                        } else {
                            BenchSupport.addMechanicalModule(world, a, b, i);
                        }
                    }
                    default -> throw new IllegalArgumentException("Unknown mix: " + constraintTypeMix);
                }
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
    public void stepConstraintWorld(BenchState state, Blackhole bh) {
        state.world.step(1f / 60f, 1);
        BodyState sample = state.world.getBodyState(state.bodies.get(0));
        PhysicsStats stats = state.world.getStats();
        bh.consume(sample.position().x());
        bh.consume(sample.angularVelocity().y());
        bh.consume(stats.constraintCount());
    }
}
