package org.dynamisphysics.jolt;

import com.github.stephengold.joltjni.BroadPhaseLayerInterfaceTable;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.JobSystemThreadPool;
import com.github.stephengold.joltjni.NarrowPhaseQuery;
import com.github.stephengold.joltjni.ObjectLayerPairFilterTable;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilterTable;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.TempAllocatorImpl;
import com.github.stephengold.joltjni.TempAllocatorMalloc;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.ShapeType;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.config.PhysicsTuningResolver;
import org.dynamisphysics.api.config.ResolvedTuning;
import org.dynamisphysics.api.config.AllocatorMode;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.ContactListener;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.query.ShapecastResult;
import org.dynamisphysics.api.world.CharacterState;
import org.dynamisphysics.api.world.PhysicsStats;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.dynamisphysics.jolt.character.JoltCharacterController;
import org.dynamisphysics.jolt.constraint.JoltConstraintRegistry;
import org.dynamisphysics.jolt.constraint.JoltMechanicalConstraintController;
import org.dynamisphysics.jolt.event.JoltContactListener;
import org.dynamisphysics.jolt.event.JoltEventBuffer;
import org.dynamisphysics.jolt.query.JoltRaycastExecutor;
import org.dynamisphysics.jolt.ragdoll.JoltRagdollSystem;
import org.dynamisphysics.jolt.snapshot.JoltSnapshot;
import org.dynamisphysics.jolt.vehicle.JoltVehicleSystem;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.dynamisphysics.jolt.world.JoltConversions.toRVec3;
import static org.dynamisphysics.jolt.world.JoltConversions.toVec3;

public final class JoltPhysicsWorld implements PhysicsWorld {
    private static final int NUM_OBJECT_LAYERS = 256;
    private static final int NUM_BROAD_PHASE_LAYERS = 1;
    private static volatile boolean nativeLoaded;
    private static volatile boolean runtimeInitialized;
    private static final boolean TRACE_STEP = Boolean.getBoolean("jolt.trace");

    private final PhysicsWorldConfig config;
    private final ResolvedTuning resolvedTuning;
    private final PhysicsSystem physicsSystem;
    private final TempAllocator allocator;
    private final JobSystemThreadPool jobs;
    private final JoltBodyRegistry bodyRegistry;
    private final JoltEventBuffer eventBuffer;
    private final JoltRaycastExecutor raycastExecutor;
    private final JoltConstraintRegistry constraintRegistry;
    private final JoltMechanicalConstraintController mechanicalConstraintController;
    private final JoltVehicleSystem vehicleSystem;
    private final JoltCharacterController characterController;
    private final JoltRagdollSystem ragdollSystem;
    private final List<ContactListener> contactListeners = new ArrayList<>();
    private Vector3f gravity;

    private int stepCount;
    private boolean paused;
    private boolean destroyed;
    private float timeScale = 1f;
    private float lastStepMs;

    private JoltPhysicsWorld(
        PhysicsWorldConfig config,
        ResolvedTuning resolvedTuning,
        PhysicsSystem physicsSystem,
        TempAllocator allocator,
        JobSystemThreadPool jobs,
        JoltBodyRegistry bodyRegistry,
        JoltEventBuffer eventBuffer,
        JoltRaycastExecutor raycastExecutor,
        JoltConstraintRegistry constraintRegistry,
        JoltMechanicalConstraintController mechanicalConstraintController,
        JoltVehicleSystem vehicleSystem,
        JoltCharacterController characterController,
        JoltRagdollSystem ragdollSystem
    ) {
        this.config = config;
        this.resolvedTuning = resolvedTuning;
        this.physicsSystem = physicsSystem;
        this.allocator = allocator;
        this.jobs = jobs;
        this.bodyRegistry = bodyRegistry;
        this.eventBuffer = eventBuffer;
        this.raycastExecutor = raycastExecutor;
        this.constraintRegistry = constraintRegistry;
        this.mechanicalConstraintController = mechanicalConstraintController;
        this.vehicleSystem = vehicleSystem;
        this.characterController = characterController;
        this.ragdollSystem = ragdollSystem;
        this.gravity = new Vector3f(config.gravity());
    }

    public static JoltPhysicsWorld create(PhysicsWorldConfig config) {
        ResolvedTuning resolved = PhysicsTuningResolver.resolve(config);
        ensureRuntimeInitialized();

        PhysicsSystem physics = new PhysicsSystem();
        BroadPhaseLayerInterfaceTable broad = new BroadPhaseLayerInterfaceTable(NUM_OBJECT_LAYERS, NUM_BROAD_PHASE_LAYERS);
        for (int i = 0; i < NUM_OBJECT_LAYERS; i++) {
            broad.mapObjectToBroadPhaseLayer(i, 0);
        }

        ObjectLayerPairFilterTable pair = new ObjectLayerPairFilterTable(NUM_OBJECT_LAYERS);
        for (int i = 0; i < NUM_OBJECT_LAYERS; i++) {
            for (int j = 0; j < NUM_OBJECT_LAYERS; j++) {
                pair.enableCollision(i, j);
            }
        }

        ObjectVsBroadPhaseLayerFilterTable objectVsBroad =
            new ObjectVsBroadPhaseLayerFilterTable(broad, NUM_OBJECT_LAYERS, pair, NUM_BROAD_PHASE_LAYERS);

        physics.init(
            config.maxBodies(),
            0,
            config.maxConstraints(),
            config.maxConstraints(),
            broad,
            objectVsBroad,
            pair
        );
        physics.setGravity(config.gravity().x(), config.gravity().y(), config.gravity().z());

        TempAllocator allocator = resolved.allocatorMode() == AllocatorMode.IMPL
            ? new TempAllocatorImpl(resolved.allocatorBytes())
            : new TempAllocatorMalloc();
        if (resolved.profile() == org.dynamisphysics.api.config.PhysicsTuningProfile.PERF
            && resolved.allocatorMode() == AllocatorMode.IMPL
            && resolved.allocatorBytes() < (16 * 1024 * 1024)) {
            System.err.println(
                "DYNAMIS-TUNING PERF with IMPL allocator below 16MB may increase native allocation pressure: "
                    + (resolved.allocatorBytes() / (1024 * 1024)) + "MB"
            );
        }
        int threadCount = resolved.threads();
        JobSystemThreadPool jobs = new JobSystemThreadPool(Jolt.cMaxPhysicsJobs, Jolt.cMaxPhysicsBarriers, threadCount);
        trace("jobs.threads=" + threadCount
            + " deterministic=" + resolved.deterministic()
            + " allocator=" + resolved.allocatorMode()
            + " allocatorBytes=" + resolved.allocatorBytes()
            + " solverIterations=" + resolved.solverIterations());

        JoltBodyRegistry bodyRegistry = new JoltBodyRegistry(physics.getBodyInterface());
        JoltEventBuffer eventBuffer = new JoltEventBuffer();
        physics.setContactListener(new JoltContactListener(bodyRegistry, eventBuffer));
        NarrowPhaseQuery query = (NarrowPhaseQuery) physics.getNarrowPhaseQuery();
        JoltRaycastExecutor raycastExecutor = new JoltRaycastExecutor(query, physics.getBodyInterface(), bodyRegistry);
        JoltConstraintRegistry constraintRegistry = new JoltConstraintRegistry(physics, bodyRegistry);
        JoltMechanicalConstraintController mechanicalConstraintController =
            new JoltMechanicalConstraintController(physics.getBodyInterface(), bodyRegistry, constraintRegistry);
        JoltVehicleSystem vehicleSystem = new JoltVehicleSystem(physics, bodyRegistry, eventBuffer);
        JoltCharacterController characterController =
            new JoltCharacterController(physics, bodyRegistry, raycastExecutor, eventBuffer, allocator);
        JoltRagdollSystem ragdollSystem = new JoltRagdollSystem(physics, bodyRegistry);

        return new JoltPhysicsWorld(
            config, resolved, physics, allocator, jobs, bodyRegistry, eventBuffer, raycastExecutor, constraintRegistry,
            mechanicalConstraintController,
            vehicleSystem, characterController, ragdollSystem
        );
    }

    @Override
    public void step(float deltaSeconds) {
        step(deltaSeconds, config.maxSubSteps());
    }

    @Override
    public void step(float deltaSeconds, int subSteps) {
        ensureNotDestroyed();
        if (paused) {
            return;
        }
        if (!(deltaSeconds > 0f) || Float.isNaN(deltaSeconds) || Float.isInfinite(deltaSeconds)) {
            throw new IllegalArgumentException("deltaSeconds must be > 0 and finite, got " + deltaSeconds);
        }
        if (subSteps <= 0) {
            throw new IllegalArgumentException("subSteps must be > 0, got " + subSteps);
        }
        validateInitialized();
        int clamped = subSteps;
        long start = System.nanoTime();
        trace("step.enter dt=" + deltaSeconds + " subSteps=" + subSteps + " timeScale=" + timeScale);
        vehicleSystem.stepAll(deltaSeconds * timeScale);
        characterController.stepAll(deltaSeconds * timeScale, gravity);
        trace("step.before-update");
        physicsSystem.update(deltaSeconds * timeScale, clamped, allocator, jobs);
        trace("step.after-update");
        mechanicalConstraintController.postSolve(deltaSeconds * timeScale);
        ragdollSystem.stepAll(deltaSeconds * timeScale);
        lastStepMs = (System.nanoTime() - start) / 1_000_000f;
        stepCount++;
        trace("step.exit stepCount=" + stepCount);
    }

    @Override
    public void pause() {
        paused = true;
    }

    @Override
    public void resume() {
        paused = false;
    }

    @Override
    public void destroy() {
        if (destroyed) {
            return;
        }
        destroyed = true;
        eventBuffer.clear();
        ragdollSystem.clearAll();
        vehicleSystem.clearAll();
        characterController.clearAll();
        constraintRegistry.clearAll();
        bodyRegistry.clearAllBodies();
        physicsSystem.destroyAllBodies();
    }

    @Override
    public RigidBodyHandle spawnRigidBody(RigidBodyConfig config) {
        ensureNotDestroyed();
        return bodyRegistry.spawn(config);
    }

    @Override
    public void destroyRigidBody(RigidBodyHandle handle) {
        ensureNotDestroyed();
        bodyRegistry.destroy(handle);
    }

    @Override
    public BodyState getBodyState(RigidBodyHandle handle) {
        ensureNotDestroyed();
        return bodyRegistry.getState(handle);
    }

    @Override
    public void setBodyState(RigidBodyHandle handle, BodyState state) {
        ensureNotDestroyed();
        bodyRegistry.setState(handle, state);
    }

    @Override
    public void applyImpulse(RigidBodyHandle h, Vector3f impulse, Vector3f worldPoint) {
        ensureNotDestroyed();
        bodyRegistry.applyImpulse(h, toVec3(impulse), toRVec3(worldPoint));
    }

    @Override
    public void applyForce(RigidBodyHandle h, Vector3f force, Vector3f worldPoint) {
        ensureNotDestroyed();
        bodyRegistry.applyForce(h, toVec3(force), toRVec3(worldPoint));
    }

    @Override
    public void applyTorque(RigidBodyHandle h, Vector3f torque) {
        ensureNotDestroyed();
        bodyRegistry.applyTorque(h, toVec3(torque));
    }

    @Override
    public void setVelocity(RigidBodyHandle h, Vector3f linear, Vector3f angular) {
        ensureNotDestroyed();
        bodyRegistry.setVelocity(h, toVec3(linear), toVec3(angular));
    }

    @Override
    public void teleport(RigidBodyHandle h, Vector3f position, Quaternionf orientation) {
        ensureNotDestroyed();
        bodyRegistry.setState(h, new BodyState(position, orientation, new Vector3f(), new Vector3f(), false));
    }

    @Override
    public ConstraintHandle addConstraint(ConstraintDesc desc) {
        ensureNotDestroyed();
        return constraintRegistry.add(desc);
    }

    @Override
    public void removeConstraint(ConstraintHandle handle) {
        ensureNotDestroyed();
        constraintRegistry.remove(handle);
    }

    @Override
    public void setConstraintEnabled(ConstraintHandle h, boolean enabled) {
        ensureNotDestroyed();
        constraintRegistry.setEnabled(h, enabled);
    }

    @Override
    public void setMotorTarget(ConstraintHandle h, float targetVelocityOrPosition) {
        ensureNotDestroyed();
        constraintRegistry.setMotorTarget(h, targetVelocityOrPosition);
    }

    @Override
    public VehicleHandle spawnVehicle(VehicleDescriptor desc) {
        ensureNotDestroyed();
        return vehicleSystem.spawn(desc);
    }

    @Override
    public void destroyVehicle(VehicleHandle handle) {
        ensureNotDestroyed();
        vehicleSystem.destroy(handle);
    }

    @Override
    public void applyThrottle(VehicleHandle h, float throttle) {
        ensureNotDestroyed();
        vehicleSystem.applyThrottle(h, throttle);
    }

    @Override
    public void applyBrake(VehicleHandle h, float brake) {
        ensureNotDestroyed();
        vehicleSystem.applyBrake(h, brake);
    }

    @Override
    public void applySteering(VehicleHandle h, float steeringAngle) {
        ensureNotDestroyed();
        vehicleSystem.applySteering(h, steeringAngle);
    }

    @Override
    public void applyHandbrake(VehicleHandle h, boolean engaged) {
        ensureNotDestroyed();
        vehicleSystem.applyHandbrake(h, engaged);
    }

    @Override
    public VehicleState getVehicleState(VehicleHandle h) {
        ensureNotDestroyed();
        return vehicleSystem.getVehicleState(h);
    }

    @Override
    public CharacterHandle spawnCharacter(CharacterDescriptor desc) {
        ensureNotDestroyed();
        return characterController.spawn(desc);
    }

    @Override
    public void destroyCharacter(CharacterHandle handle) {
        ensureNotDestroyed();
        characterController.destroy(handle);
    }

    @Override
    public void moveCharacter(CharacterHandle h, Vector3f velocity) {
        ensureNotDestroyed();
        characterController.move(h, velocity);
    }

    @Override
    public void jumpCharacter(CharacterHandle h, float impulse) {
        ensureNotDestroyed();
        characterController.jump(h, impulse);
    }

    @Override
    public CharacterState getCharacterState(CharacterHandle h) {
        ensureNotDestroyed();
        return characterController.getState(h);
    }

    @Override
    public RagdollHandle spawnRagdoll(RagdollDescriptor desc, AnimisPose initialPose) {
        ensureNotDestroyed();
        return ragdollSystem.spawn(desc, initialPose);
    }

    @Override
    public void destroyRagdoll(RagdollHandle handle) {
        ensureNotDestroyed();
        ragdollSystem.destroy(handle);
    }

    @Override
    public void activateRagdoll(RagdollHandle h, float blendInSeconds) {
        ensureNotDestroyed();
        ragdollSystem.activate(h, blendInSeconds);
    }

    @Override
    public void deactivateRagdoll(RagdollHandle h) {
        ensureNotDestroyed();
        ragdollSystem.deactivate(h);
    }

    @Override
    public void setRagdollBlendTarget(RagdollHandle h, AnimisPose pose, float alpha) {
        ensureNotDestroyed();
        ragdollSystem.setBlendTarget(h, pose, alpha);
    }

    @Override
    public Optional<RaycastResult> raycastClosest(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        ensureNotDestroyed();
        return raycastExecutor.raycastClosest(origin, dir, maxDist, layerMask);
    }

    @Override
    public List<RaycastResult> raycastAll(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        ensureNotDestroyed();
        return raycastExecutor.raycastAll(origin, dir, maxDist, layerMask);
    }

    @Override
    public List<RigidBodyHandle> overlapSphere(Vector3f centre, float radius, int layerMask) {
        return List.of();
    }

    @Override
    public List<RigidBodyHandle> overlapAabb(Vector3f min, Vector3f max, int layerMask) {
        return List.of();
    }

    @Override
    public Optional<ShapecastResult> shapecast(CollisionShape shape, Vector3f from, Vector3f to, int layerMask) {
        return Optional.empty();
    }

    @Override
    public void addContactListener(ContactListener listener) {
        contactListeners.add(listener);
    }

    @Override
    public void removeContactListener(ContactListener listener) {
        contactListeners.remove(listener);
    }

    @Override
    public List<PhysicsEvent> drainEvents() {
        ensureNotDestroyed();
        List<PhysicsEvent> events = eventBuffer.drain();
        events.stream()
            .filter(e -> e instanceof ContactEvent)
            .map(e -> (ContactEvent) e)
            .forEach(e -> contactListeners.forEach(l -> l.onContact(e)));
        return events;
    }

    @Override
    public byte[] snapshot() {
        ensureNotDestroyed();
        return JoltSnapshot.write(stepCount, config.gravity(), timeScale, bodyRegistry.bodiesInIdOrder(), bodyRegistry::getState);
    }

    @Override
    public void restore(byte[] snapshot) {
        ensureNotDestroyed();
        JoltSnapshot.RestoredState restored = JoltSnapshot.read(snapshot);
        bodyRegistry.clearAllBodies();
        physicsSystem.setGravity(restored.gravity().x(), restored.gravity().y(), restored.gravity().z());

        restored.bodies().stream()
            .sorted(java.util.Comparator.comparingInt(JoltSnapshot.BodySnapshot::bodyId))
            .forEach(body -> {
                CollisionShape shape = fromShapeSnapshot(body.shape());
                BodyState state = body.state();
                Matrix4f transform = new Matrix4f().identity().translation(
                    state.position().x(),
                    state.position().y(),
                    state.position().z()
                );
                RigidBodyConfig config = RigidBodyConfig.builder(shape, body.mass())
                    .mode(body.mode())
                    .layer(body.layer())
                    .collidesWith(body.collidesWith())
                    .gravityScale(body.gravityScale())
                    .worldTransform(transform)
                    .linearVelocity(state.linearVelocity())
                    .angularVelocity(state.angularVelocity())
                    .build();
                JoltBodyHandle handle = bodyRegistry.spawnWithId(config, body.bodyId());
                bodyRegistry.setState(handle, state);
            });

        this.stepCount = restored.stepCount();
        this.timeScale = restored.timeScale();
    }

    @Override
    public void setGravity(Vector3f gravity) {
        ensureNotDestroyed();
        physicsSystem.setGravity(gravity.x(), gravity.y(), gravity.z());
        this.gravity = new Vector3f(gravity);
    }

    @Override
    public void setTimeScale(float scale) {
        this.timeScale = scale;
    }

    @Override
    public PhysicsStats getStats() {
        ensureNotDestroyed();
        int total = bodyRegistry.bodyCount();
        int active = (int) bodyRegistry.allHandles().stream()
            .filter(h -> h.mode() == org.dynamisphysics.api.body.BodyMode.DYNAMIC)
            .filter(h -> physicsSystem.getBodyInterface().isActive(h.joltBodyId()))
            .count();
        return new PhysicsStats(lastStepMs, total, active, Math.max(0, total - active), 0, 0, 0f, 0f, 0f, 0f);
    }

    private static CollisionShape fromShapeSnapshot(JoltSnapshot.ShapeSnapshot shape) {
        ShapeType type = ShapeType.values()[shape.type()];
        float[] f = shape.f();
        return switch (type) {
            case SPHERE -> CollisionShape.sphere(f[0]);
            case BOX -> CollisionShape.box(f[0], f[1], f[2]);
            case CAPSULE -> CollisionShape.capsule(f[0], f[1]);
            case CYLINDER -> CollisionShape.cylinder(f[0], f[1]);
            case PLANE -> CollisionShape.plane(f[0], f[1], f[2], f[3]);
            default -> throw new UnsupportedOperationException("Shape restore not supported: " + type);
        };
    }

    private static void ensureNativeLoaded() {
        if (nativeLoaded) {
            return;
        }
        synchronized (JoltPhysicsWorld.class) {
            if (nativeLoaded) {
                return;
            }
            String[] resourceCandidates = {
                "osx/aarch64/com/github/stephengold/libjoltjni.dylib",
                "linux/x86_64/com/github/stephengold/libjoltjni.so",
                "windows/x86_64/com/github/stephengold/joltjni.dll"
            };
            List<String> loadErrors = new ArrayList<>();
            for (String candidate : resourceCandidates) {
                String error = tryLoadFromResource(candidate);
                if (error == null) {
                    nativeLoaded = true;
                    return;
                }
                loadErrors.add(candidate + " -> " + error);
            }
            throw new IllegalStateException(
                "Unable to load jolt-jni native library from bundled resources. "
                    + "Checked: " + String.join("; ", loadErrors)
            );
        }
    }

    private void validateInitialized() {
        if (physicsSystem == null || allocator == null || jobs == null || bodyRegistry == null) {
            throw new IllegalStateException("Jolt world not fully initialised");
        }
    }

    private void ensureNotDestroyed() {
        if (destroyed) {
            throw new IllegalStateException("JoltPhysicsWorld is destroyed");
        }
    }

    private static void trace(String message) {
        if (TRACE_STEP) {
            System.err.println("JOLT-TRACE " + message);
        }
    }

    private static void ensureRuntimeInitialized() {
        if (runtimeInitialized) {
            return;
        }
        synchronized (JoltPhysicsWorld.class) {
            if (runtimeInitialized) {
                return;
            }
            ensureNativeLoaded();
            Jolt.registerDefaultAllocator();
            Jolt.newFactory();
            Jolt.registerTypes();
            runtimeInitialized = true;
        }
    }

    private static String tryLoadFromResource(String resourcePath) {
        ClassLoader loader = JoltPhysicsWorld.class.getClassLoader();
        try (InputStream in = loader.getResourceAsStream(resourcePath)) {
            if (in == null) {
                return "resource-not-found";
            }
            String suffix = resourcePath.endsWith(".dll") ? ".dll"
                : resourcePath.endsWith(".so") ? ".so" : ".dylib";
            Path temp = Files.createTempFile("joltjni-", suffix);
            temp.toFile().deleteOnExit();
            Files.copy(in, temp, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
            System.load(temp.toAbsolutePath().toString());
            return null;
        } catch (IOException | UnsatisfiedLinkError ex) {
            return ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }

    ResolvedTuning resolvedTuningForTesting() {
        return resolvedTuning;
    }
}
