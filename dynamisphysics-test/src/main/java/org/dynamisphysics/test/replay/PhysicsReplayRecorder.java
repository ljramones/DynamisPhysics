package org.dynamisphysics.test.replay;

import org.dynamiscollision.shapes.CollisionShape;
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
import org.dynamisphysics.api.body.StableRigidBodyId;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.ResolvedTuning;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.event.ContactListener;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.query.ShapecastResult;
import org.dynamisphysics.api.world.CharacterState;
import org.dynamisphysics.api.world.PhysicsStats;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Base64;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

public final class PhysicsReplayRecorder implements PhysicsWorld {
    private final PhysicsWorld delegate;
    private final String engineVersion;
    private final PhysicsBackend backend;
    private final ResolvedTuning tuning;
    private final float fixedTimeStep;
    private final int maxSubSteps;
    private final ReproPacket.ReproScene scene;
    private final long seed;

    private final Map<Integer, List<ReplayOp>> opsByStep = new LinkedHashMap<>();
    private final List<ReplayCheckpoint> checkpoints = new ArrayList<>();
    private final IdentityHashMap<RigidBodyHandle, Integer> rigidBodyIds = new IdentityHashMap<>();
    private final IdentityHashMap<ConstraintHandle, Integer> constraintIds = new IdentityHashMap<>();
    private final IdentityHashMap<VehicleHandle, Integer> vehicleIds = new IdentityHashMap<>();
    private final IdentityHashMap<CharacterHandle, Integer> characterIds = new IdentityHashMap<>();
    private final IdentityHashMap<RagdollHandle, Integer> ragdollIds = new IdentityHashMap<>();
    private int nextRigidBodyId = 1;
    private int nextConstraintId = 1;
    private int nextVehicleId = 1;
    private int nextCharacterId = 1;
    private int nextRagdollId = 1;
    private int currentStep;
    private int checkpointEverySteps;
    private String initialSnapshotB64;

    public PhysicsReplayRecorder(
        PhysicsWorld delegate,
        String engineVersion,
        PhysicsBackend backend,
        ResolvedTuning tuning,
        float fixedTimeStep,
        int maxSubSteps,
        String sceneName,
        Map<String, Object> sceneParams,
        long seed
    ) {
        this.delegate = delegate;
        this.engineVersion = engineVersion;
        this.backend = backend;
        this.tuning = tuning;
        this.fixedTimeStep = fixedTimeStep;
        this.maxSubSteps = maxSubSteps;
        this.scene = new ReproPacket.ReproScene(sceneName, sceneParams);
        this.seed = seed;
    }

    public void setCheckpointEverySteps(int checkpointEverySteps) {
        this.checkpointEverySteps = Math.max(0, checkpointEverySteps);
    }

    public void captureInitialSnapshot() {
        this.initialSnapshotB64 = Base64.getEncoder().encodeToString(delegate.snapshot());
    }

    public ReproPacket buildPacket() {
        if (initialSnapshotB64 == null) {
            captureInitialSnapshot();
        }
        List<ReplayInputFrame> frames = opsByStep.entrySet().stream()
            .map(e -> ReplayOp.frame(e.getKey(), e.getValue()))
            .toList();
        return ReproPacket.of(
            engineVersion,
            backend,
            tuning,
            fixedTimeStep,
            maxSubSteps,
            scene,
            resolveValidationMode(),
            ReplayInvariants.defaults(),
            seed,
            initialSnapshotB64,
            frames,
            checkpoints
        );
    }

    public PhysicsWorld delegateWorld() {
        return delegate;
    }

    private ReplayValidationMode resolveValidationMode() {
        String raw = System.getProperty("physics.replay.validationMode");
        if (raw != null && !raw.isBlank()) {
            try {
                return ReplayValidationMode.valueOf(raw.trim().toUpperCase(Locale.ROOT));
            } catch (IllegalArgumentException ignored) {
                // fall through to deterministic heuristic
            }
        }
        return tuning.deterministic() && backend == PhysicsBackend.ODE4J
            ? ReplayValidationMode.STRICT
            : ReplayValidationMode.BEHAVIOURAL;
    }

    private void maybeCheckpoint() {
        if (checkpointEverySteps > 0 && currentStep > 0 && currentStep % checkpointEverySteps == 0) {
            checkpoints.add(new ReplayCheckpoint(currentStep, ReplayHash.sha256Hex(delegate.snapshot())));
        }
    }

    private void record(ReplayOp op) {
        opsByStep.computeIfAbsent(currentStep, ignored -> new ArrayList<>()).add(op);
    }

    private int requireRigidBodyId(RigidBodyHandle handle) {
        Integer id = rigidBodyIds.get(handle);
        if (id != null) {
            return id;
        }
        Integer stable = handle instanceof StableRigidBodyId s ? s.bodyId() : null;
        if (stable != null) {
            rigidBodyIds.put(handle, stable);
            return stable;
        }
        throw new IllegalArgumentException("Handle not registered in recorder: " + handle);
    }

    private int requireVehicleId(VehicleHandle handle) {
        Integer id = vehicleIds.get(handle);
        if (id == null) {
            throw new IllegalArgumentException("Vehicle handle not registered in recorder: " + handle);
        }
        return id;
    }

    private int requireCharacterId(CharacterHandle handle) {
        Integer id = characterIds.get(handle);
        if (id == null) {
            throw new IllegalArgumentException("Character handle not registered in recorder: " + handle);
        }
        return id;
    }

    private int requireRagdollId(RagdollHandle handle) {
        Integer id = ragdollIds.get(handle);
        if (id == null) {
            throw new IllegalArgumentException("Ragdoll handle not registered in recorder: " + handle);
        }
        return id;
    }

    @Override
    public void step(float deltaSeconds) {
        delegate.step(deltaSeconds);
        currentStep++;
        maybeCheckpoint();
    }

    @Override
    public void step(float deltaSeconds, int subSteps) {
        delegate.step(deltaSeconds, subSteps);
        currentStep++;
        maybeCheckpoint();
    }

    @Override
    public void pause() {
        delegate.pause();
    }

    @Override
    public void resume() {
        delegate.resume();
    }

    @Override
    public void destroy() {
        delegate.destroy();
    }

    @Override
    public RigidBodyHandle spawnRigidBody(RigidBodyConfig config) {
        RigidBodyHandle handle = delegate.spawnRigidBody(config);
        Integer stable = handle instanceof StableRigidBodyId s ? s.bodyId() : null;
        rigidBodyIds.put(handle, stable != null ? stable : nextRigidBodyId++);
        return handle;
    }

    @Override
    public void destroyRigidBody(RigidBodyHandle handle) {
        delegate.destroyRigidBody(handle);
    }

    @Override
    public BodyState getBodyState(RigidBodyHandle handle) {
        return delegate.getBodyState(handle);
    }

    @Override
    public void setBodyState(RigidBodyHandle handle, BodyState state) {
        delegate.setBodyState(handle, state);
    }

    @Override
    public void applyImpulse(RigidBodyHandle h, Vector3f impulse, Vector3f worldPoint) {
        delegate.applyImpulse(h, impulse, worldPoint);
        record(new ReplayOp.ApplyImpulseOp(requireRigidBodyId(h), ReplayOp.Vec3.of(impulse), ReplayOp.Vec3.of(worldPoint)));
    }

    @Override
    public void applyForce(RigidBodyHandle h, Vector3f force, Vector3f worldPoint) {
        delegate.applyForce(h, force, worldPoint);
        record(new ReplayOp.ApplyForceOp(requireRigidBodyId(h), ReplayOp.Vec3.of(force), ReplayOp.Vec3.of(worldPoint)));
    }

    @Override
    public void applyTorque(RigidBodyHandle h, Vector3f torque) {
        delegate.applyTorque(h, torque);
        record(new ReplayOp.ApplyTorqueOp(requireRigidBodyId(h), ReplayOp.Vec3.of(torque)));
    }

    @Override
    public void setVelocity(RigidBodyHandle h, Vector3f linear, Vector3f angular) {
        delegate.setVelocity(h, linear, angular);
        record(new ReplayOp.SetVelocityOp(requireRigidBodyId(h), ReplayOp.Vec3.of(linear), ReplayOp.Vec3.of(angular)));
    }

    @Override
    public void teleport(RigidBodyHandle h, Vector3f position, Quaternionf orientation) {
        delegate.teleport(h, position, orientation);
        record(new ReplayOp.TeleportOp(requireRigidBodyId(h), ReplayOp.Vec3.of(position), ReplayOp.Quat.of(orientation)));
    }

    @Override
    public ConstraintHandle addConstraint(ConstraintDesc desc) {
        ConstraintHandle handle = delegate.addConstraint(desc);
        constraintIds.put(handle, nextConstraintId++);
        return handle;
    }

    @Override
    public void removeConstraint(ConstraintHandle handle) {
        delegate.removeConstraint(handle);
    }

    @Override
    public void setConstraintEnabled(ConstraintHandle h, boolean enabled) {
        delegate.setConstraintEnabled(h, enabled);
    }

    @Override
    public void setMotorTarget(ConstraintHandle h, float targetVelocityOrPosition) {
        delegate.setMotorTarget(h, targetVelocityOrPosition);
    }

    @Override
    public VehicleHandle spawnVehicle(VehicleDescriptor desc) {
        VehicleHandle handle = delegate.spawnVehicle(desc);
        vehicleIds.put(handle, nextVehicleId++);
        return handle;
    }

    @Override
    public void destroyVehicle(VehicleHandle handle) {
        delegate.destroyVehicle(handle);
    }

    @Override
    public void applyThrottle(VehicleHandle h, float throttle) {
        delegate.applyThrottle(h, throttle);
        record(new ReplayOp.ApplyThrottleOp(requireVehicleId(h), throttle));
    }

    @Override
    public void applyBrake(VehicleHandle h, float brake) {
        delegate.applyBrake(h, brake);
        record(new ReplayOp.ApplyBrakeOp(requireVehicleId(h), brake));
    }

    @Override
    public void applySteering(VehicleHandle h, float steeringAngle) {
        delegate.applySteering(h, steeringAngle);
        record(new ReplayOp.ApplySteeringOp(requireVehicleId(h), steeringAngle));
    }

    @Override
    public void applyHandbrake(VehicleHandle h, boolean engaged) {
        delegate.applyHandbrake(h, engaged);
        record(new ReplayOp.ApplyHandbrakeOp(requireVehicleId(h), engaged));
    }

    @Override
    public VehicleState getVehicleState(VehicleHandle h) {
        return delegate.getVehicleState(h);
    }

    @Override
    public CharacterHandle spawnCharacter(CharacterDescriptor desc) {
        CharacterHandle handle = delegate.spawnCharacter(desc);
        characterIds.put(handle, nextCharacterId++);
        return handle;
    }

    @Override
    public void destroyCharacter(CharacterHandle handle) {
        delegate.destroyCharacter(handle);
    }

    @Override
    public void moveCharacter(CharacterHandle h, Vector3f velocity) {
        delegate.moveCharacter(h, velocity);
        record(new ReplayOp.MoveCharacterOp(requireCharacterId(h), ReplayOp.Vec3.of(velocity)));
    }

    @Override
    public void jumpCharacter(CharacterHandle h, float impulse) {
        delegate.jumpCharacter(h, impulse);
        record(new ReplayOp.JumpCharacterOp(requireCharacterId(h), impulse));
    }

    @Override
    public CharacterState getCharacterState(CharacterHandle h) {
        return delegate.getCharacterState(h);
    }

    @Override
    public RagdollHandle spawnRagdoll(RagdollDescriptor desc, AnimisPose initialPose) {
        RagdollHandle handle = delegate.spawnRagdoll(desc, initialPose);
        ragdollIds.put(handle, nextRagdollId++);
        return handle;
    }

    @Override
    public void destroyRagdoll(RagdollHandle handle) {
        delegate.destroyRagdoll(handle);
    }

    @Override
    public void activateRagdoll(RagdollHandle h, float blendInSeconds) {
        delegate.activateRagdoll(h, blendInSeconds);
        record(new ReplayOp.ActivateRagdollOp(requireRagdollId(h), blendInSeconds));
    }

    @Override
    public void deactivateRagdoll(RagdollHandle h) {
        delegate.deactivateRagdoll(h);
        record(new ReplayOp.DeactivateRagdollOp(requireRagdollId(h)));
    }

    @Override
    public void setRagdollBlendTarget(RagdollHandle h, AnimisPose pose, float alpha) {
        delegate.setRagdollBlendTarget(h, pose, alpha);
        String poseHint = pose == null ? "null" : pose.getClass().getName();
        record(new ReplayOp.SetRagdollBlendTargetOp(requireRagdollId(h), alpha, poseHint));
    }

    @Override
    public Optional<RaycastResult> raycastClosest(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        return delegate.raycastClosest(origin, dir, maxDist, layerMask);
    }

    @Override
    public List<RaycastResult> raycastAll(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        return delegate.raycastAll(origin, dir, maxDist, layerMask);
    }

    @Override
    public List<RigidBodyHandle> overlapSphere(Vector3f centre, float radius, int layerMask) {
        return delegate.overlapSphere(centre, radius, layerMask);
    }

    @Override
    public List<RigidBodyHandle> overlapAabb(Vector3f min, Vector3f max, int layerMask) {
        return delegate.overlapAabb(min, max, layerMask);
    }

    @Override
    public Optional<ShapecastResult> shapecast(CollisionShape shape, Vector3f from, Vector3f to, int layerMask) {
        return delegate.shapecast(shape, from, to, layerMask);
    }

    @Override
    public void addContactListener(ContactListener listener) {
        delegate.addContactListener(listener);
    }

    @Override
    public void removeContactListener(ContactListener listener) {
        delegate.removeContactListener(listener);
    }

    @Override
    public List<PhysicsEvent> drainEvents() {
        return delegate.drainEvents();
    }

    @Override
    public byte[] snapshot() {
        return delegate.snapshot();
    }

    @Override
    public void restore(byte[] snapshot) {
        delegate.restore(snapshot);
    }

    @Override
    public void setGravity(Vector3f gravity) {
        delegate.setGravity(gravity);
    }

    @Override
    public void setTimeScale(float scale) {
        delegate.setTimeScale(scale);
    }

    @Override
    public PhysicsStats getStats() {
        return delegate.getStats();
    }

}
