package org.dynamisphysics.jolt.character;

import com.github.stephengold.joltjni.BodyFilter;
import com.github.stephengold.joltjni.BroadPhaseLayerFilter;
import com.github.stephengold.joltjni.CapsuleShape;
import com.github.stephengold.joltjni.CharacterVirtual;
import com.github.stephengold.joltjni.CharacterVirtualSettings;
import com.github.stephengold.joltjni.ExtendedUpdateSettings;
import com.github.stephengold.joltjni.ObjectLayerFilter;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.ShapeFilter;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.Vec3;
import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.event.FootContactEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.world.CharacterState;
import org.dynamisphysics.api.world.FootContactHint;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.dynamisphysics.jolt.event.JoltEventBuffer;
import org.dynamisphysics.jolt.query.JoltRaycastExecutor;
import org.vectrix.core.Vector3f;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import static org.dynamisphysics.jolt.world.JoltConversions.toVec3;
import static org.dynamisphysics.jolt.world.JoltConversions.toVector3f;

public final class JoltCharacterController {
    private static final Vector3f UP = new Vector3f(0f, 1f, 0f);
    private static final int MAX_OBJECT_LAYERS = 256;

    private final PhysicsSystem physicsSystem;
    private final JoltBodyRegistry bodyRegistry;
    private final JoltRaycastExecutor raycastExecutor;
    private final JoltEventBuffer eventBuffer;
    private final TempAllocator allocator;

    private final ExtendedUpdateSettings updateSettings = new ExtendedUpdateSettings();
    private final BroadPhaseLayerFilter broadPhaseLayerFilter = new BroadPhaseLayerFilter();
    private final ObjectLayerFilter objectLayerFilter = new ObjectLayerFilter();
    private final BodyFilter bodyFilter = new BodyFilter();
    private final ShapeFilter shapeFilter = new ShapeFilter();
    private final Map<CharacterHandle, JoltCharacterHandle> characters = new LinkedHashMap<>();

    public JoltCharacterController(
        PhysicsSystem physicsSystem,
        JoltBodyRegistry bodyRegistry,
        JoltRaycastExecutor raycastExecutor,
        JoltEventBuffer eventBuffer,
        TempAllocator allocator
    ) {
        this.physicsSystem = physicsSystem;
        this.bodyRegistry = bodyRegistry;
        this.raycastExecutor = raycastExecutor;
        this.eventBuffer = eventBuffer;
        this.allocator = allocator;
    }

    public CharacterHandle spawn(CharacterDescriptor descriptor) {
        float radius = org.vectrix.core.Math.max(0.02f, descriptor.radius());
        float height = org.vectrix.core.Math.max(radius * 2f, descriptor.height());
        float halfHeightOfCylinder = org.vectrix.core.Math.max(0f, (height - radius * 2f) * 0.5f);

        CharacterVirtualSettings settings = new CharacterVirtualSettings();
        settings.setShape(new CapsuleShape(halfHeightOfCylinder, radius));
        settings.setUp(new Vec3(0f, 1f, 0f));
        settings.setMaxSlopeAngle((float) java.lang.Math.toRadians(descriptor.maxSlopeAngleDeg()));
        settings.setCharacterPadding(org.vectrix.core.Math.max(0.01f, descriptor.skinWidth()));
        settings.setMass(org.vectrix.core.Math.max(1f, descriptor.mass()));
        settings.setMaxStrength(org.vectrix.core.Math.max(1f, descriptor.pushForce()));
        settings.setInnerBodyLayer(normalizeLayer(descriptor.layer()));

        float startY = descriptor.height() * 0.5f + descriptor.stepHeight() + 0.2f;
        RVec3 startPosition = new RVec3(0d, startY, 0d);
        CharacterVirtual character = new CharacterVirtual(
            settings,
            startPosition,
            new Quat(0f, 0f, 0f, 1f),
            0L,
            physicsSystem
        );

        JoltCharacterHandle handle = new JoltCharacterHandle(descriptor, character, new Vector3f(0f, startY, 0f));
        characters.put(handle, handle);
        return handle;
    }

    public void destroy(CharacterHandle handle) {
        JoltCharacterHandle ch = characters.remove(handle);
        if (ch != null) {
            ch.kill();
        }
    }

    public void move(CharacterHandle handle, Vector3f velocity) {
        ch(handle).intendedVelocity = new Vector3f(velocity);
    }

    public void jump(CharacterHandle handle, float impulse) {
        JoltCharacterHandle ch = ch(handle);
        ch.pendingJumpImpulse = org.vectrix.core.Math.max(0f, impulse);
    }

    public CharacterState getState(CharacterHandle handle) {
        return ch(handle).getState();
    }

    public void clearAll() {
        List<CharacterHandle> snapshot = List.copyOf(characters.keySet());
        for (CharacterHandle handle : snapshot) {
            destroy(handle);
        }
    }

    public void stepAll(float dt, Vector3f gravity) {
        for (JoltCharacterHandle character : characters.values()) {
            stepCharacter(character, dt, gravity);
        }
    }

    private void stepCharacter(JoltCharacterHandle handle, float dt, Vector3f gravity) {
        if (!handle.isAlive()) {
            return;
        }
        CharacterState previous = handle.getState();
        CharacterDescriptor desc = handle.descriptor();

        CharacterVirtual character = handle.character();
        Vector3f inheritedVelocity = previous.isGrounded()
            ? toVector3f(character.getGroundVelocity())
            : new Vector3f();
        if (handle.pendingJumpImpulse > 0f && previous.isGrounded()) {
            handle.verticalVelocity = handle.pendingJumpImpulse;
            handle.pendingJumpImpulse = 0f;
        } else if (!previous.isGrounded()) {
            handle.verticalVelocity += gravity.y() * dt;
        } else {
            handle.verticalVelocity = 0f;
        }

        character.setLinearVelocity(new Vec3(
            handle.intendedVelocity.x() + inheritedVelocity.x(),
            handle.verticalVelocity,
            handle.intendedVelocity.z() + inheritedVelocity.z()
        ));
        updateSettings.setWalkStairsStepUp(new Vec3(0f, desc.stepHeight(), 0f));
        updateSettings.setStickToFloorStepDown(
            new Vec3(0f, desc.stepHeight() + org.vectrix.core.Math.max(0.01f, desc.skinWidth()), 0f)
        );
        character.extendedUpdate(
            dt,
            toVec3(gravity),
            updateSettings,
            broadPhaseLayerFilter,
            objectLayerFilter,
            bodyFilter,
            shapeFilter,
            allocator
        );

        boolean grounded = character.isSupported();
        Vector3f position = toVector3f(character.getPosition());
        Vector3f velocity = toVector3f(character.getLinearVelocity());
        handle.verticalVelocity = velocity.y();
        Vector3f groundNormal = grounded ? toVector3f(character.getGroundNormal()) : new Vector3f(UP);

        JoltBodyHandle groundBodyHandle = bodyRegistry.getByJoltId(character.getGroundBodyId());
        PhysicsMaterial groundMaterial = groundBodyHandle != null
            ? groundBodyHandle.config().material()
            : PhysicsMaterial.DEFAULT;
        Vector3f groundVelocity = grounded ? toVector3f(character.getGroundVelocity()) : new Vector3f();

        GroundHit fallbackGround = sampleGround(position, desc);
        if (!grounded && fallbackGround.hit()) {
            grounded = true;
            groundNormal = fallbackGround.normal();
            groundMaterial = fallbackGround.material();
            groundBodyHandle = fallbackGround.body();
            groundVelocity = fallbackGround.body() != null
                ? bodyRegistry.getState(fallbackGround.body()).linearVelocity()
                : new Vector3f();
        }
        if (grounded && handle.verticalVelocity < 0f) {
            handle.verticalVelocity = 0f;
        }

        CharacterState next = new CharacterState(
            position,
            velocity,
            grounded,
            groundMaterial,
            groundNormal,
            groundBodyHandle,
            groundVelocity
        );
        handle.setState(next);

        if (!handle.lastGrounded && grounded) {
            eventBuffer.add(new FootContactEvent(
                new Vector3f(position),
                new Vector3f(groundNormal),
                groundMaterial,
                org.vectrix.core.Math.abs(previous.velocity().y())
            ));
        }

        if (desc.footIkListener() != null) {
            if (grounded) {
                desc.footIkListener().accept(FootContactHint.grounded(
                    "foot",
                    new Vector3f(position),
                    new Vector3f(groundNormal),
                    groundMaterial.tag()
                ));
            } else if (handle.lastGrounded) {
                desc.footIkListener().accept(FootContactHint.airborne("foot"));
            }
        }

        handle.lastGrounded = grounded;
    }

    private GroundHit sampleGround(Vector3f position, CharacterDescriptor desc) {
        float rayStartOffset = desc.radius() + desc.stepHeight();
        float maxDistance = desc.height() * 0.5f + desc.stepHeight() + desc.skinWidth() + 0.35f;
        Vector3f origin = new Vector3f(position.x(), position.y() + rayStartOffset, position.z());
        Optional<RaycastResult> hit = raycastExecutor.raycastClosest(
            origin, new Vector3f(0f, -1f, 0f), maxDistance, desc.collidesWith()
        );
        if (hit.isEmpty()) {
            return GroundHit.none();
        }
        RaycastResult h = hit.get();
        float maxSlopeCos = (float) java.lang.Math.cos(java.lang.Math.toRadians(desc.maxSlopeAngleDeg()));
        if (h.normal().dot(UP) < maxSlopeCos) {
            return GroundHit.none();
        }
        JoltBodyHandle body = h.body() instanceof JoltBodyHandle jh ? jh : null;
        return new GroundHit(true, new Vector3f(h.normal()), h.material(), body);
    }

    private JoltCharacterHandle ch(CharacterHandle handle) {
        JoltCharacterHandle c = characters.get(handle);
        if (c == null) {
            throw new IllegalArgumentException("Unknown character handle: " + handle);
        }
        return c;
    }

    private static int normalizeLayer(int layer) {
        return Math.floorMod(layer, MAX_OBJECT_LAYERS);
    }

    private record GroundHit(
        boolean hit,
        Vector3f normal,
        PhysicsMaterial material,
        JoltBodyHandle body
    ) {
        static GroundHit none() {
            return new GroundHit(false, new Vector3f(0f, 1f, 0f), PhysicsMaterial.DEFAULT, null);
        }
    }
}
