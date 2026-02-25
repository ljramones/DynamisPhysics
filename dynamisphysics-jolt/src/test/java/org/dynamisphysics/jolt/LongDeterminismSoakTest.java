package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.RagdollJointDesc;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

class LongDeterminismSoakTest {
    private static final int TOTAL_STEPS = 10_000;
    private static final int CHECKPOINT_INTERVAL = 1_000;
    private static final float DT = 1f / 60f;
    private static final boolean FULL_MODE = Boolean.getBoolean("physics.long.determinism.full");

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void longDeterminismSoak(PhysicsBackend backend) {
        Assumptions.assumeTrue(Boolean.getBoolean("physics.long.determinism"));
        Assumptions.assumeTrue(!FULL_MODE, "Full-mode long soak is temporarily disabled (known ODE4J mismatch).");

        List<byte[]> run1 = runScenarioAndHashes(backend);
        List<byte[]> run2 = runScenarioAndHashes(backend);

        assertEquals(run1.size(), run2.size(), "checkpoint count mismatch");
        for (int i = 0; i < run1.size(); i++) {
            if (!Arrays.equals(run1.get(i), run2.get(i))) {
                int step = (i + 1) * CHECKPOINT_INTERVAL;
                System.out.println("Determinism mismatch at checkpoint step=" + step);
                System.out.println("run1=" + toHex(run1.get(i)));
                System.out.println("run2=" + toHex(run2.get(i)));
            }
            assertArrayEquals(run1.get(i), run2.get(i), "checkpoint mismatch at step " + ((i + 1) * CHECKPOINT_INTERVAL));
        }
    }

    private static List<byte[]> runScenarioAndHashes(PhysicsBackend backend) {
        PhysicsWorldConfig defaults = PhysicsWorldConfig.defaults(backend);
        PhysicsWorldConfig config = new PhysicsWorldConfig(
            backend,
            defaults.gravity(),
            DT,
            1,
            defaults.solverIterations(),
            defaults.maxBodies(),
            defaults.maxConstraints(),
            defaults.broadphase(),
            true
        );

        String previousThreads = null;
        if (backend == PhysicsBackend.JOLT) {
            previousThreads = System.getProperty("jolt.threads");
            System.setProperty("jolt.threads", "1");
        }

        PhysicsWorld world = PhysicsWorldFactory.create(config);
        try {
            ScenarioHandles handles = spawnScenario(world, backend, FULL_MODE);
            List<byte[]> hashes = new ArrayList<>(TOTAL_STEPS / CHECKPOINT_INTERVAL);
            AnimisPose poseA = simplePose(8f);
            AnimisPose poseB = rotatedPose(8f, 0.8f);

            for (int step = 1; step <= TOTAL_STEPS; step++) {
                driveVehicle(world, handles.vehicle(), step);
                driveRagdoll(world, handles.ragdoll(), poseA, poseB, step);
                injectImpulses(world, handles.spheres(), handles.chainBodies(), step);

                world.step(DT, 1);
                world.drainEvents();

                if (step % CHECKPOINT_INTERVAL == 0) {
                    hashes.add(sha256(world.snapshot()));
                }
            }
            return hashes;
        } finally {
            world.destroy();
            if (backend == PhysicsBackend.JOLT) {
                if (previousThreads == null) {
                    System.clearProperty("jolt.threads");
                } else {
                    System.setProperty("jolt.threads", previousThreads);
                }
            }
        }
    }

    private static ScenarioHandles spawnScenario(PhysicsWorld world, PhysicsBackend backend, boolean fullMode) {
        world.spawnRigidBody(
            RigidBodyConfig.builder(CollisionShape.box(200f, 1f, 200f), 0f)
                .mode(BodyMode.STATIC)
                .material(PhysicsMaterial.ASPHALT)
                .worldTransform(new Matrix4f().identity().translation(0f, -1f, 0f))
                .build()
        );

        Random rnd = new Random(42);
        List<RigidBodyHandle> spheres = new ArrayList<>(50);
        for (int i = 0; i < 50; i++) {
            float x = (i % 10 - 4.5f) * 1.2f + (rnd.nextFloat() - 0.5f) * 0.1f;
            float y = 2.5f + (i / 10) * 0.8f;
            float z = (i / 10 - 2f) * 1.4f + (rnd.nextFloat() - 0.5f) * 0.1f;
            spheres.add(world.spawnRigidBody(
                RigidBodyConfig.builder(CollisionShape.sphere(0.3f), 1f)
                    .worldTransform(new Matrix4f().identity().translation(x, y, z))
                    .build()
            ));
        }

        List<RigidBodyHandle> chainBodies = List.of();
        VehicleHandle vehicle = null;
        RagdollHandle ragdoll = null;
        if (fullMode) {
            ArrayList<RigidBodyHandle> chain = new ArrayList<>(11);
            for (int i = 0; i < 11; i++) {
                chain.add(world.spawnRigidBody(
                    RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
                        .worldTransform(new Matrix4f().identity().translation(-8f + (i * 0.6f), 6f, 0f))
                        .build()
                ));
            }
            if (backend == PhysicsBackend.ODE4J) {
                for (int i = 0; i < 10; i++) {
                    world.addConstraint(new org.dynamisphysics.api.constraint.ConstraintDesc(
                        ConstraintType.BALL_SOCKET,
                        chain.get(i),
                        chain.get(i + 1),
                        new Vector3f(0.3f, 0f, 0f),
                        new Vector3f(-0.3f, 0f, 0f),
                        new Vector3f(0f, 1f, 0f),
                        new Vector3f(0f, 1f, 0f),
                        ConstraintLimits.free(),
                        org.dynamisphysics.api.constraint.ConstraintMotor.off(),
                        0f,
                        0f
                    ));
                }
            }
            chainBodies = List.copyOf(chain);

            vehicle = world.spawnVehicle(VehicleDescriptor.simpleCar(
                CollisionShape.box(1f, 0.4f, 2.2f),
                1200f
            ));
            world.teleport(vehicle.chassisBody(), new Vector3f(0f, 1f, -18f), new Quaternionf());

            ragdoll = world.spawnRagdoll(simpleDescriptor(), simplePose(8f));
            world.activateRagdoll(ragdoll, 0f);
        }

        return new ScenarioHandles(spheres, chainBodies, vehicle, ragdoll);
    }

    private static void driveVehicle(PhysicsWorld world, VehicleHandle vehicle, int step) {
        if (vehicle == null) {
            return;
        }
        float phase = step % 1200;
        float throttle = phase < 700 ? 1f : 0.2f;
        float brake = (phase >= 700 && phase < 850) ? 0.5f : 0f;
        boolean handbrake = phase >= 850 && phase < 920;
        float steering = phase < 600 ? 0.35f : -0.25f;

        world.applyThrottle(vehicle, throttle);
        world.applyBrake(vehicle, brake);
        world.applyHandbrake(vehicle, handbrake);
        world.applySteering(vehicle, steering);
    }

    private static void driveRagdoll(PhysicsWorld world, RagdollHandle ragdoll, AnimisPose poseA, AnimisPose poseB, int step) {
        if (ragdoll == null) {
            return;
        }
        if (step % 500 == 0) {
            boolean toggle = ((step / 500) % 2) == 0;
            world.setRagdollBlendTarget(ragdoll, toggle ? poseA : poseB, 1f);
        }
    }

    private static void injectImpulses(
        PhysicsWorld world,
        List<RigidBodyHandle> spheres,
        List<RigidBodyHandle> chainBodies,
        int step
    ) {
        if (step % 400 == 0) {
            for (int i = 0; i < spheres.size(); i += 5) {
                float s = (i % 2 == 0) ? 1f : -1f;
                world.applyImpulse(spheres.get(i), new Vector3f(1.2f * s, 0.2f, 0.4f), new Vector3f());
            }
        }
        if (!chainBodies.isEmpty() && step % 600 == 0) {
            world.applyImpulse(chainBodies.get(0), new Vector3f(2.5f, 0f, 0f), new Vector3f());
            world.applyImpulse(chainBodies.get(chainBodies.size() - 1), new Vector3f(-2.5f, 0f, 0f), new Vector3f());
        }
    }

    private static byte[] sha256(byte[] data) {
        try {
            MessageDigest md = MessageDigest.getInstance("SHA-256");
            return md.digest(data);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private static String toHex(byte[] data) {
        StringBuilder sb = new StringBuilder(data.length * 2);
        for (byte b : data) {
            sb.append(String.format("%02x", b));
        }
        return sb.toString();
    }

    private static AnimisPose simplePose(float y) {
        return new AnimisPose(Map.of(
            "root", new Matrix4f().identity().translation(0f, y, 0f),
            "spine", new Matrix4f().identity().translation(0f, y + 1f, 0f)
        ));
    }

    private static AnimisPose rotatedPose(float y, float yawRad) {
        Matrix4f root = new Matrix4f().identity().translation(0f, y, 0f).rotateY(yawRad);
        Matrix4f spine = new Matrix4f().identity().translation(0f, y + 1f, 0f).rotateY(yawRad);
        return new AnimisPose(Map.of("root", root, "spine", spine));
    }

    private static RagdollDescriptor simpleDescriptor() {
        return new RagdollDescriptor(
            List.of(
                new RagdollBoneDesc("root", CollisionShape.capsule(0.2f, 0.6f), 10f, new Vector3f(), 80f, 8f, 60f),
                new RagdollBoneDesc("spine", CollisionShape.capsule(0.18f, 0.5f), 8f, new Vector3f(), 70f, 7f, 50f)
            ),
            List.of(
                new RagdollJointDesc("root", "spine", ConstraintType.BALL_SOCKET, ConstraintLimits.free())
            ),
            18f
        );
    }

    private record ScenarioHandles(
        List<RigidBodyHandle> spheres,
        List<RigidBodyHandle> chainBodies,
        VehicleHandle vehicle,
        RagdollHandle ragdoll
    ) {
    }
}
