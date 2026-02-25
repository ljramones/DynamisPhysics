package org.dynamisphysics.test.harness;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public final class PhysicsSimHarness {
    private PhysicsSimHarness() {}

    public record SimResult(List<PhysicsEvent> allEvents, int totalSteps, long totalStepTimeNs) {}

    public static SimResult run(PhysicsWorld world, int steps, float dt) {
        return run(world, steps, dt, w -> {});
    }

    public static SimResult run(PhysicsWorld world, int steps, float dt, Consumer<PhysicsWorld> perStepInput) {
        var events = new ArrayList<PhysicsEvent>();
        long start = System.nanoTime();
        for (int i = 0; i < steps; i++) {
            perStepInput.accept(world);
            world.step(dt);
            events.addAll(world.drainEvents());
        }
        return new SimResult(List.copyOf(events), steps, System.nanoTime() - start);
    }

    public static byte[][] runAndSnapshotTwice(PhysicsWorld world, int steps, float dt) {
        for (int i = 0; i < steps; i++) {
            world.step(dt);
        }
        byte[] snap1 = world.snapshot();
        world.restore(snap1);
        for (int i = 0; i < steps; i++) {
            world.step(dt);
        }
        byte[] snap2 = world.snapshot();
        return new byte[][]{snap1, snap2};
    }

    public static byte[] buildRunAndSnapshot(
        PhysicsWorldConfig config,
        Consumer<PhysicsWorld> setup,
        int steps,
        float dt
    ) {
        PhysicsWorld world = PhysicsWorldFactory.create(config);
        setup.accept(world);
        for (int i = 0; i < steps; i++) {
            world.step(dt);
        }
        byte[] snap = world.snapshot();
        world.destroy();
        return snap;
    }
}
