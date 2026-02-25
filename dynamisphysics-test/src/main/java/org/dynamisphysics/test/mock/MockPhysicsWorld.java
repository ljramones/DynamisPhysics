package org.dynamisphysics.test.mock;

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
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.ContactListener;
import org.dynamisphysics.api.event.ContactPoint;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.query.ShapecastResult;
import org.dynamisphysics.api.world.CharacterState;
import org.dynamisphysics.api.world.PhysicsStats;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public final class MockPhysicsWorld implements PhysicsWorld {
    private int stepCount = 0;
    private int spawnBodyCount = 0;
    private int spawnVehicleCount = 0;
    private int spawnCharCount = 0;
    private int spawnRagdollCount = 0;
    private int raycastCount = 0;
    private int applyImpulseCount = 0;

    private final Map<RigidBodyHandle, RigidBodyConfig> bodies = new LinkedHashMap<>();
    private final Map<RigidBodyHandle, BodyState> bodyStates = new LinkedHashMap<>();
    private final Map<ConstraintHandle, ConstraintDesc> constraints = new LinkedHashMap<>();
    private final List<PhysicsEvent> eventQueue = new ArrayList<>();
    private final List<ContactListener> listeners = new ArrayList<>();

    private boolean paused = false;

    public void injectEvent(PhysicsEvent event) {
        eventQueue.add(event);
    }

    public void injectContact(RigidBodyHandle a, RigidBodyHandle b, Vector3f pos, float impulse) {
        eventQueue.add(new ContactEvent(
            a,
            b,
            List.of(new ContactPoint(pos, new Vector3f(0f, 1f, 0f), 0.01f, impulse)),
            impulse,
            new Vector3f(),
            PhysicsMaterial.DEFAULT,
            PhysicsMaterial.DEFAULT
        ));
    }

    public int stepCount() { return stepCount; }
    public int spawnedBodyCount() { return spawnBodyCount; }
    public int spawnedVehicleCount() { return spawnVehicleCount; }
    public int spawnedCharCount() { return spawnCharCount; }
    public int raycastCount() { return raycastCount; }
    public int applyImpulseCount() { return applyImpulseCount; }
    public boolean isPaused() { return paused; }

    public List<RigidBodyConfig> spawnedConfigs() {
        return List.copyOf(bodies.values());
    }

    public boolean hasBody(RigidBodyHandle h) {
        return bodies.containsKey(h);
    }

    @Override public void step(float dt) { if (!paused) stepCount++; }
    @Override public void step(float dt, int subSteps) { if (!paused) stepCount += subSteps; }
    @Override public void pause() { paused = true; }
    @Override public void resume() { paused = false; }

    @Override
    public void destroy() {
        bodies.clear();
        bodyStates.clear();
        constraints.clear();
    }

    @Override
    public RigidBodyHandle spawnRigidBody(RigidBodyConfig config) {
        spawnBodyCount++;
        var handle = new MockRigidBodyHandle(config);
        bodies.put(handle, config);
        bodyStates.put(handle, defaultBodyState(config));
        return handle;
    }

    @Override
    public void destroyRigidBody(RigidBodyHandle h) {
        bodies.remove(h);
        bodyStates.remove(h);
        if (h instanceof MockRigidBodyHandle mock) {
            mock.kill();
        }
    }

    @Override
    public BodyState getBodyState(RigidBodyHandle h) {
        return bodyStates.getOrDefault(h, BodyState.ZERO);
    }

    @Override
    public void setBodyState(RigidBodyHandle h, BodyState s) {
        bodyStates.put(h, s);
    }

    @Override public void applyImpulse(RigidBodyHandle h, Vector3f i, Vector3f p) { applyImpulseCount++; }
    @Override public void applyForce(RigidBodyHandle h, Vector3f f, Vector3f p) {}
    @Override public void applyTorque(RigidBodyHandle h, Vector3f t) {}
    @Override public void setVelocity(RigidBodyHandle h, Vector3f l, Vector3f a) {}

    @Override
    public void teleport(RigidBodyHandle h, Vector3f p, Quaternionf o) {
        BodyState current = bodyStates.getOrDefault(h, BodyState.ZERO);
        bodyStates.put(h, new BodyState(p, o, current.linearVelocity(), current.angularVelocity(), current.sleeping()));
    }

    @Override
    public ConstraintHandle addConstraint(ConstraintDesc d) {
        var h = new MockConstraintHandle(d);
        constraints.put(h, d);
        return h;
    }

    @Override
    public void removeConstraint(ConstraintHandle h) {
        constraints.remove(h);
        if (h instanceof MockConstraintHandle mock) {
            mock.kill();
        }
    }

    @Override public void setConstraintEnabled(ConstraintHandle h, boolean e) {}
    @Override public void setMotorTarget(ConstraintHandle h, float t) {}

    @Override
    public VehicleHandle spawnVehicle(VehicleDescriptor d) {
        spawnVehicleCount++;
        return new MockVehicleHandle();
    }

    @Override
    public void destroyVehicle(VehicleHandle h) {
        if (h instanceof MockVehicleHandle mock) {
            mock.kill();
        }
    }

    @Override public void applyThrottle(VehicleHandle h, float t) {}
    @Override public void applyBrake(VehicleHandle h, float b) {}
    @Override public void applySteering(VehicleHandle h, float s) {}
    @Override public void applyHandbrake(VehicleHandle h, boolean e) {}
    @Override public VehicleState getVehicleState(VehicleHandle h) { return VehicleState.ZERO; }

    @Override
    public CharacterHandle spawnCharacter(CharacterDescriptor d) {
        spawnCharCount++;
        return new MockCharacterHandle();
    }

    @Override
    public void destroyCharacter(CharacterHandle h) {
        if (h instanceof MockCharacterHandle mock) {
            mock.kill();
        }
    }

    @Override public void moveCharacter(CharacterHandle h, Vector3f v) {}
    @Override public void jumpCharacter(CharacterHandle h, float i) {}
    @Override public CharacterState getCharacterState(CharacterHandle h) { return CharacterState.ZERO; }

    @Override
    public RagdollHandle spawnRagdoll(RagdollDescriptor d, AnimisPose p) {
        spawnRagdollCount++;
        return new MockRagdollHandle();
    }

    @Override
    public void destroyRagdoll(RagdollHandle h) {
        if (h instanceof MockRagdollHandle mock) {
            mock.kill();
        }
    }

    @Override public void activateRagdoll(RagdollHandle h, float s) {}
    @Override public void deactivateRagdoll(RagdollHandle h) {}
    @Override public void setRagdollBlendTarget(RagdollHandle h, AnimisPose p, float a) {}

    @Override
    public Optional<RaycastResult> raycastClosest(Vector3f o, Vector3f d, float m, int l) {
        raycastCount++;
        return Optional.empty();
    }

    @Override
    public List<RaycastResult> raycastAll(Vector3f o, Vector3f d, float m, int l) {
        raycastCount++;
        return List.of();
    }

    @Override public List<RigidBodyHandle> overlapSphere(Vector3f c, float r, int l) { return List.of(); }
    @Override public List<RigidBodyHandle> overlapAabb(Vector3f min, Vector3f max, int l) { return List.of(); }

    @Override
    public Optional<ShapecastResult> shapecast(CollisionShape s, Vector3f f, Vector3f t, int l) {
        return Optional.empty();
    }

    @Override public void addContactListener(ContactListener l) { listeners.add(l); }
    @Override public void removeContactListener(ContactListener l) { listeners.remove(l); }

    @Override
    public List<PhysicsEvent> drainEvents() {
        var copy = List.copyOf(eventQueue);
        eventQueue.clear();
        return copy;
    }

    @Override public byte[] snapshot() { return new byte[0]; }
    @Override public void restore(byte[] snap) {}
    @Override public void setGravity(Vector3f g) {}
    @Override public void setTimeScale(float s) {}

    @Override
    public PhysicsStats getStats() {
        return new PhysicsStats(0f, bodies.size(), bodies.size(), 0, constraints.size(), 1, 0f, 0f, 0f, 0f);
    }

    private static BodyState defaultBodyState(RigidBodyConfig config) {
        Vector3f pos = new Vector3f();
        config.worldTransform().getTranslation(pos);
        Quaternionf ori = new Quaternionf();
        config.worldTransform().getUnnormalizedRotation(ori);
        return new BodyState(pos, ori, config.linearVelocity(), config.angularVelocity(), false);
    }
}
