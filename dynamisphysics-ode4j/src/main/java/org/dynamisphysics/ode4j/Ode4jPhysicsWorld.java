package org.dynamisphysics.ode4j;

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
import org.dynamisphysics.api.config.PhysicsWorldConfig;
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
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.dynamisphysics.ode4j.body.Ode4jForceAccumulator;
import org.dynamisphysics.ode4j.character.Ode4jCharacterController;
import org.dynamisphysics.ode4j.constraint.Ode4jConstraintRegistry;
import org.dynamisphysics.ode4j.event.Ode4jContactDispatcher;
import org.dynamisphysics.ode4j.event.Ode4jEventBuffer;
import org.dynamisphysics.ode4j.query.Ode4jRaycastExecutor;
import org.dynamisphysics.ode4j.vehicle.Ode4jVehicleSystem;
import org.dynamisphysics.ode4j.world.Ode4jStepLoop;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public final class Ode4jPhysicsWorld implements PhysicsWorld {
    private final PhysicsWorldConfig config;
    private final DWorld world;
    private final DHashSpace space;
    private final DJointGroup contactGroup;
    private final Ode4jBodyRegistry bodyRegistry;
    private final Ode4jForceAccumulator forceAccumulator;
    private final Ode4jConstraintRegistry constraintRegistry;
    private final Ode4jEventBuffer eventBuffer;
    private final Ode4jContactDispatcher dispatcher;
    private final Ode4jStepLoop stepLoop;
    private final Ode4jRaycastExecutor raycastExecutor;
    private final Ode4jVehicleSystem vehicleSystem;
    private final Ode4jCharacterController characterController;
    private final List<ContactListener> contactListeners = new ArrayList<>();

    private boolean paused = false;
    private float timeScale = 1f;

    private Ode4jPhysicsWorld(
        PhysicsWorldConfig config,
        DWorld world,
        DHashSpace space,
        DJointGroup contactGroup,
        Ode4jBodyRegistry bodyRegistry,
        Ode4jForceAccumulator forceAccumulator,
        Ode4jConstraintRegistry constraintRegistry,
        Ode4jEventBuffer eventBuffer,
        Ode4jContactDispatcher dispatcher,
        Ode4jStepLoop stepLoop,
        Ode4jRaycastExecutor raycastExecutor,
        Ode4jVehicleSystem vehicleSystem,
        Ode4jCharacterController characterController
    ) {
        this.config = config;
        this.world = world;
        this.space = space;
        this.contactGroup = contactGroup;
        this.bodyRegistry = bodyRegistry;
        this.forceAccumulator = forceAccumulator;
        this.constraintRegistry = constraintRegistry;
        this.eventBuffer = eventBuffer;
        this.dispatcher = dispatcher;
        this.stepLoop = stepLoop;
        this.raycastExecutor = raycastExecutor;
        this.vehicleSystem = vehicleSystem;
        this.characterController = characterController;
    }

    public static Ode4jPhysicsWorld create(PhysicsWorldConfig config) {
        OdeHelper.initODE2(0);
        DWorld world = OdeHelper.createWorld();
        DHashSpace space = OdeHelper.createHashSpace(null);
        DJointGroup contactGroup = OdeHelper.createJointGroup();

        world.setGravity(config.gravity().x(), config.gravity().y(), config.gravity().z());
        world.setERP(0.2);
        world.setCFM(1e-5);
        world.setQuickStepNumIterations(config.solverIterations());
        world.setAutoDisableFlag(true);
        world.setAutoDisableLinearThreshold(0.01);
        world.setAutoDisableAngularThreshold(0.01);
        world.setAutoDisableSteps(10);

        var eventBuffer = new Ode4jEventBuffer();
        var forceAccumulator = new Ode4jForceAccumulator();
        var bodyRegistry = new Ode4jBodyRegistry(world, space);
        var constraintRegistry = new Ode4jConstraintRegistry(world, bodyRegistry, eventBuffer);
        var dispatcher = new Ode4jContactDispatcher(world, contactGroup, eventBuffer);
        var raycastExecutor = new Ode4jRaycastExecutor(space);
        var vehicleSystem = new Ode4jVehicleSystem(bodyRegistry, eventBuffer, raycastExecutor);
        var characterController = new Ode4jCharacterController(bodyRegistry, raycastExecutor, eventBuffer);
        var stepLoop = new Ode4jStepLoop(
            world,
            space,
            contactGroup,
            forceAccumulator,
            dispatcher,
            constraintRegistry,
            vehicleSystem,
            characterController
        );

        return new Ode4jPhysicsWorld(config, world, space, contactGroup, bodyRegistry, forceAccumulator,
            constraintRegistry,
            eventBuffer, dispatcher, stepLoop, raycastExecutor, vehicleSystem, characterController);
    }

    @Override public void step(float dt) { step(dt, config.maxSubSteps()); }
    @Override public void step(float dt, int subSteps) { if (!paused) stepLoop.step(dt * timeScale, subSteps); }
    @Override public void pause() { paused = true; }
    @Override public void resume() { paused = false; }

    @Override
    public void destroy() {
        contactGroup.destroy();
        space.destroy();
        world.destroy();
        OdeHelper.closeODE();
    }

    @Override public RigidBodyHandle spawnRigidBody(RigidBodyConfig config) { return bodyRegistry.spawn(config); }
    @Override public void destroyRigidBody(RigidBodyHandle h) { bodyRegistry.destroy(h); }
    @Override public BodyState getBodyState(RigidBodyHandle h) { return bodyRegistry.getState(h); }
    @Override public void setBodyState(RigidBodyHandle h, BodyState s) { bodyRegistry.setState(h, s); }

    @Override
    public void applyImpulse(RigidBodyHandle h, Vector3f impulse, Vector3f point) {
        Ode4jBodyHandle oh = (Ode4jBodyHandle) h;
        if (oh.body() != null) forceAccumulator.addImpulse(oh.body(), impulse, point);
    }

    @Override
    public void applyForce(RigidBodyHandle h, Vector3f force, Vector3f point) {
        Ode4jBodyHandle oh = (Ode4jBodyHandle) h;
        if (oh.body() != null) forceAccumulator.addForce(oh.body(), force, point);
    }

    @Override
    public void applyTorque(RigidBodyHandle h, Vector3f torque) {
        Ode4jBodyHandle oh = (Ode4jBodyHandle) h;
        if (oh.body() != null) forceAccumulator.addTorque(oh.body(), torque);
    }

    @Override
    public void setVelocity(RigidBodyHandle h, Vector3f linear, Vector3f angular) {
        Ode4jBodyHandle oh = (Ode4jBodyHandle) h;
        if (oh.body() != null) forceAccumulator.setVelocity(oh.body(), linear, angular);
    }

    @Override
    public void teleport(RigidBodyHandle h, Vector3f pos, Quaternionf ori) {
        bodyRegistry.setState(h, new BodyState(pos, ori, new Vector3f(), new Vector3f(), false));
    }

    @Override public ConstraintHandle addConstraint(ConstraintDesc d) { return constraintRegistry.add(d); }
    @Override public void removeConstraint(ConstraintHandle h) { constraintRegistry.remove(h); }
    @Override public void setConstraintEnabled(ConstraintHandle h, boolean e) { constraintRegistry.setEnabled(h, e); }
    @Override public void setMotorTarget(ConstraintHandle h, float t) { constraintRegistry.setMotorTarget(h, t); }

    @Override public VehicleHandle spawnVehicle(VehicleDescriptor d) { return vehicleSystem.spawn(d); }
    @Override public void destroyVehicle(VehicleHandle h) { vehicleSystem.destroy(h); }
    @Override public void applyThrottle(VehicleHandle h, float t) { vehicleSystem.applyThrottle(h, t); }
    @Override public void applyBrake(VehicleHandle h, float b) { vehicleSystem.applyBrake(h, b); }
    @Override public void applySteering(VehicleHandle h, float s) { vehicleSystem.applySteering(h, s); }
    @Override public void applyHandbrake(VehicleHandle h, boolean e) { vehicleSystem.applyHandbrake(h, e); }
    @Override public VehicleState getVehicleState(VehicleHandle h) { return vehicleSystem.getVehicleState(h); }

    @Override public CharacterHandle spawnCharacter(CharacterDescriptor d) { return characterController.spawn(d); }
    @Override public void destroyCharacter(CharacterHandle h) { characterController.destroy(h); }
    @Override public void moveCharacter(CharacterHandle h, Vector3f v) { characterController.move(h, v); }
    @Override public void jumpCharacter(CharacterHandle h, float i) { characterController.jump(h, i); }
    @Override public CharacterState getCharacterState(CharacterHandle h) { return characterController.getState(h); }

    @Override public RagdollHandle spawnRagdoll(RagdollDescriptor d, AnimisPose p) { throw new UnsupportedOperationException("Step 8"); }
    @Override public void destroyRagdoll(RagdollHandle h) {}
    @Override public void activateRagdoll(RagdollHandle h, float s) {}
    @Override public void deactivateRagdoll(RagdollHandle h) {}
    @Override public void setRagdollBlendTarget(RagdollHandle h, AnimisPose p, float a) {}

    @Override
    public Optional<RaycastResult> raycastClosest(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        return raycastExecutor.raycastClosest(origin, dir, maxDist, layerMask);
    }

    @Override
    public List<RaycastResult> raycastAll(Vector3f origin, Vector3f dir, float maxDist, int layerMask) {
        return raycastExecutor.raycastAll(origin, dir, maxDist, layerMask);
    }

    @Override public List<RigidBodyHandle> overlapSphere(Vector3f c, float r, int l) { return List.of(); }
    @Override public List<RigidBodyHandle> overlapAabb(Vector3f min, Vector3f max, int l) { return List.of(); }
    @Override public Optional<ShapecastResult> shapecast(CollisionShape s, Vector3f f, Vector3f t, int l) { return Optional.empty(); }

    @Override public void addContactListener(ContactListener l) { contactListeners.add(l); }
    @Override public void removeContactListener(ContactListener l) { contactListeners.remove(l); }

    @Override
    public List<PhysicsEvent> drainEvents() {
        var events = eventBuffer.drain();
        events.stream().filter(e -> e instanceof ContactEvent).map(e -> (ContactEvent) e)
            .forEach(e -> contactListeners.forEach(l -> l.onContact(e)));
        return events;
    }

    @Override public byte[] snapshot() { return new byte[0]; }
    @Override public void restore(byte[] snap) {}

    @Override public void setGravity(Vector3f g) { world.setGravity(g.x(), g.y(), g.z()); }
    @Override public void setTimeScale(float s) { timeScale = s; }

    @Override
    public PhysicsStats getStats() {
        int total = bodyRegistry.bodyCount();
        int active = countActive();
        return new PhysicsStats(
            stepLoop.lastStepMs(),
            total,
            active,
            total - active,
            constraintRegistry.constraintCount(),
            0,
            0f,
            0f,
            0f,
            0f
        );
    }

    private int countActive() {
        return (int) bodyRegistry.allHandles().stream()
            .filter(h -> h.body() != null && h.body().isEnabled())
            .count();
    }
}
