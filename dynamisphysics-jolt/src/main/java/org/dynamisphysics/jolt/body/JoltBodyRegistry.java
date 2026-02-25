package org.dynamisphysics.jolt.body;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Shape;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.jolt.shape.JoltShapeAdapter;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.dynamisphysics.jolt.world.JoltConversions.toQuat;
import static org.dynamisphysics.jolt.world.JoltConversions.toRVec3;
import static org.dynamisphysics.jolt.world.JoltConversions.toVec3;
import static org.dynamisphysics.jolt.world.JoltConversions.toVector3f;
import static org.dynamisphysics.jolt.world.JoltConversions.toQuaternionf;

public final class JoltBodyRegistry {
    private static final int MAX_OBJECT_LAYERS = 256;

    private final BodyInterface bodyInterface;
    private final Map<RigidBodyHandle, JoltBodyHandle> byHandle = new LinkedHashMap<>();
    private final Map<Integer, JoltBodyHandle> byStableId = new LinkedHashMap<>();
    private final Map<Integer, JoltBodyHandle> byJoltId = new HashMap<>();
    private int nextBodyId = 1;

    public JoltBodyRegistry(BodyInterface bodyInterface) {
        this.bodyInterface = bodyInterface;
    }

    public JoltBodyHandle spawn(RigidBodyConfig config) {
        return spawnWithId(config, nextBodyId++);
    }

    public JoltBodyHandle spawnWithId(RigidBodyConfig config, int stableBodyId) {
        nextBodyId = Math.max(nextBodyId, stableBodyId + 1);
        Shape shape = JoltShapeAdapter.toJoltShape(config.shape());

        BodyCreationSettings settings = new BodyCreationSettings(
            shape,
            toRVec3(extractPosition(config)),
            toQuat(extractOrientation(config)),
            toMotionType(config.mode()),
            normalizeLayer(config.layer())
        );
        settings.setLinearVelocity(toVec3(config.linearVelocity()));
        settings.setAngularVelocity(toVec3(config.angularVelocity()));
        settings.setFriction(config.material().friction());
        settings.setRestitution(config.material().restitution());
        settings.setIsSensor(config.isSensor() || config.mode() == BodyMode.SENSOR);
        settings.setGravityFactor(config.gravityScale());
        settings.setUserData(stableBodyId);

        EActivation activation = config.mode() == BodyMode.STATIC
            ? EActivation.DontActivate : EActivation.Activate;
        int joltId = bodyInterface.createAndAddBody(settings, activation);
        JoltBodyHandle handle = new JoltBodyHandle(stableBodyId, joltId, config);
        byHandle.put(handle, handle);
        byStableId.put(stableBodyId, handle);
        byJoltId.put(joltId, handle);
        return handle;
    }

    public void destroy(RigidBodyHandle handle) {
        JoltBodyHandle jh = byHandle.remove(handle);
        if (jh == null) {
            return;
        }
        byStableId.remove(jh.bodyId());
        byJoltId.remove(jh.joltBodyId());
        bodyInterface.removeBody(jh.joltBodyId());
        bodyInterface.destroyBody(jh.joltBodyId());
        jh.kill();
    }

    public BodyState getState(RigidBodyHandle handle) {
        JoltBodyHandle jh = byHandle.get(handle);
        if (jh == null || !jh.isAlive()) {
            return BodyState.ZERO;
        }

        RVec3 position = bodyInterface.getPosition(jh.joltBodyId());
        Quat rotation = bodyInterface.getRotation(jh.joltBodyId());
        Vec3 linear = bodyInterface.getLinearVelocity(jh.joltBodyId());
        Vec3 angular = bodyInterface.getAngularVelocity(jh.joltBodyId());
        boolean sleeping = !bodyInterface.isActive(jh.joltBodyId());

        return new BodyState(
            toVector3f(position),
            toQuaternionf(rotation),
            toVector3f(linear),
            toVector3f(angular),
            sleeping
        );
    }

    public void setState(RigidBodyHandle handle, BodyState state) {
        JoltBodyHandle jh = byHandle.get(handle);
        if (jh == null || !jh.isAlive()) {
            return;
        }
        int bodyId = jh.joltBodyId();
        bodyInterface.setPositionAndRotation(bodyId, toRVec3(state.position()), toQuat(state.orientation()), EActivation.Activate);
        bodyInterface.setLinearAndAngularVelocity(bodyId, toVec3(state.linearVelocity()), toVec3(state.angularVelocity()));
    }

    public void applyImpulse(RigidBodyHandle handle, Vec3 impulse, RVec3 atPoint) {
        JoltBodyHandle jh = byHandle.get(handle);
        if (jh != null && jh.isAlive()) {
            bodyInterface.addImpulse(jh.joltBodyId(), impulse, atPoint);
        }
    }

    public void applyForce(RigidBodyHandle handle, Vec3 force, RVec3 atPoint) {
        JoltBodyHandle jh = byHandle.get(handle);
        if (jh != null && jh.isAlive()) {
            bodyInterface.addForce(jh.joltBodyId(), force, atPoint);
        }
    }

    public void applyTorque(RigidBodyHandle handle, Vec3 torque) {
        JoltBodyHandle jh = byHandle.get(handle);
        if (jh != null && jh.isAlive()) {
            bodyInterface.addTorque(jh.joltBodyId(), torque);
        }
    }

    public void setVelocity(RigidBodyHandle handle, Vec3 linear, Vec3 angular) {
        JoltBodyHandle jh = byHandle.get(handle);
        if (jh != null && jh.isAlive()) {
            bodyInterface.setLinearAndAngularVelocity(jh.joltBodyId(), linear, angular);
        }
    }

    public Collection<JoltBodyHandle> allHandles() {
        return Collections.unmodifiableCollection(byHandle.values());
    }

    public int bodyCount() {
        return byHandle.size();
    }

    public JoltBodyHandle getByHandle(RigidBodyHandle handle) {
        return byHandle.get(handle);
    }

    public JoltBodyHandle getByStableId(int stableId) {
        return byStableId.get(stableId);
    }

    public JoltBodyHandle getByJoltId(int joltId) {
        return byJoltId.get(joltId);
    }

    public List<JoltBodyHandle> bodiesInIdOrder() {
        List<JoltBodyHandle> ordered = new ArrayList<>(byStableId.values());
        ordered.sort(Comparator.comparingInt(JoltBodyHandle::bodyId));
        return ordered;
    }

    public void clearAllBodies() {
        List<RigidBodyHandle> snapshot = List.copyOf(byHandle.keySet());
        for (RigidBodyHandle handle : snapshot) {
            destroy(handle);
        }
    }

    private static int normalizeLayer(int layer) {
        return Math.floorMod(layer, MAX_OBJECT_LAYERS);
    }

    private static Vector3f extractPosition(RigidBodyConfig config) {
        Vector3f out = new Vector3f();
        config.worldTransform().getTranslation(out);
        return out;
    }

    private static Quaternionf extractOrientation(RigidBodyConfig config) {
        Quaternionf out = new Quaternionf();
        config.worldTransform().getUnnormalizedRotation(out);
        return out;
    }

    private static EMotionType toMotionType(BodyMode mode) {
        return switch (mode) {
            case STATIC -> EMotionType.Static;
            case KINEMATIC -> EMotionType.Kinematic;
            case DYNAMIC -> EMotionType.Dynamic;
            case SENSOR -> EMotionType.Static;
        };
    }
}
