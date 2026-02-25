package org.dynamisphysics.ode4j.character;

import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.event.FootContactEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.world.CharacterState;
import org.dynamisphysics.api.world.FootContactHint;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.dynamisphysics.ode4j.event.Ode4jEventBuffer;
import org.dynamisphysics.ode4j.query.Ode4jRaycastExecutor;
import org.vectrix.core.Vector3f;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

public final class Ode4jCharacterController {
    private static final Vector3f UP = new Vector3f(0f, 1f, 0f);

    private final Ode4jBodyRegistry bodyRegistry;
    private final Ode4jRaycastExecutor raycastExecutor;
    private final Ode4jEventBuffer eventBuffer;

    private final Map<CharacterHandle, Ode4jCharacterHandle> characters = new LinkedHashMap<>();

    public Ode4jCharacterController(
        Ode4jBodyRegistry bodyRegistry,
        Ode4jRaycastExecutor raycastExecutor,
        Ode4jEventBuffer eventBuffer
    ) {
        this.bodyRegistry = bodyRegistry;
        this.raycastExecutor = raycastExecutor;
        this.eventBuffer = eventBuffer;
    }

    public CharacterHandle spawn(CharacterDescriptor descriptor) {
        float startY = descriptor.height() * 0.5f + descriptor.stepHeight() + 0.2f;
        Ode4jCharacterHandle handle = new Ode4jCharacterHandle(descriptor, new Vector3f(0f, startY, 0f));
        characters.put(handle, handle);
        return handle;
    }

    public void destroy(CharacterHandle handle) {
        Ode4jCharacterHandle c = characters.remove(handle);
        if (c != null) {
            c.kill();
        }
    }

    public void move(CharacterHandle handle, Vector3f velocity) {
        ch(handle).intendedVelocity = new Vector3f(velocity);
    }

    public void jump(CharacterHandle handle, float impulse) {
        Ode4jCharacterHandle c = ch(handle);
        c.jumpImpulse = org.vectrix.core.Math.max(impulse, 0f);
    }

    public CharacterState getState(CharacterHandle handle) {
        return ch(handle).getState();
    }

    public int characterCount() {
        return characters.size();
    }

    public void stepAll(float dt, Vector3f gravity) {
        for (Ode4jCharacterHandle character : characters.values()) {
            stepCharacter(character, dt, gravity);
        }
    }

    private void stepCharacter(Ode4jCharacterHandle character, float dt, Vector3f gravity) {
        CharacterDescriptor desc = character.descriptor();
        CharacterState prev = character.getState();

        Vector3f pos = new Vector3f(prev.position());
        Vector3f vel = new Vector3f(prev.velocity());

        Vector3f inheritedVel = new Vector3f();
        if (prev.isGrounded() && prev.groundBody() != null) {
            inheritedVel = bodyRegistry.getState(prev.groundBody()).linearVelocity();
        }

        boolean jumpedThisStep = false;
        if (character.jumpImpulse > 0f && prev.isGrounded()) {
            vel.y = character.jumpImpulse;
            character.jumpImpulse = 0f;
            jumpedThisStep = true;
            character.jumpGroundSnapCooldown = 0.12f;
        }

        vel.x = character.intendedVelocity.x() + inheritedVel.x();
        vel.z = character.intendedVelocity.z() + inheritedVel.z();
        vel.y = vel.y() + gravity.y() * dt;

        Vector3f target = pos.add(new Vector3f(vel).mul(dt), new Vector3f());

        target = tryStepUp(pos, target, desc, dt);

        character.jumpGroundSnapCooldown = org.vectrix.core.Math.max(0f, character.jumpGroundSnapCooldown - dt);
        boolean disableGroundSnap = jumpedThisStep || character.jumpGroundSnapCooldown > 0f;
        GroundHit groundHit = disableGroundSnap ? GroundHit.none() : sampleGround(target, desc);
        boolean grounded = false;
        PhysicsMaterial groundMaterial = PhysicsMaterial.DEFAULT;
        Vector3f groundNormal = new Vector3f(0f, 1f, 0f);
        if (groundHit.hit) {
            groundNormal = groundHit.normal;
            float slopeCos = groundNormal.dot(UP);
            float maxSlopeCos = (float) java.lang.Math.cos(java.lang.Math.toRadians(desc.maxSlopeAngleDeg()));
            if (slopeCos >= maxSlopeCos) {
                grounded = true;
                target.y = groundHit.position.y() + desc.height() * 0.5f + desc.skinWidth();
                vel.y = 0f;
                groundMaterial = groundHit.material;
            } else {
                Vector3f slide = projectOnPlane(gravity, groundNormal).mul(dt * 0.5f, new Vector3f());
                target.add(slide);
            }
        }

        CharacterState next = new CharacterState(
            target,
            vel,
            grounded,
            groundMaterial,
            groundNormal,
            grounded ? groundHit.body : null,
            inheritedVel
        );
        character.setState(next);

        if (grounded) {
            eventBuffer.add(new FootContactEvent(
                new Vector3f(groundHit.position),
                new Vector3f(groundNormal),
                groundMaterial,
                org.vectrix.core.Math.abs(prev.velocity().y())
            ));

            if (desc.footIkListener() != null) {
                desc.footIkListener().accept(FootContactHint.grounded(
                    "foot",
                    new Vector3f(groundHit.position),
                    new Vector3f(groundNormal),
                    groundMaterial.tag()
                ));
            }
        } else if (desc.footIkListener() != null) {
            desc.footIkListener().accept(FootContactHint.airborne("foot"));
        }
    }

    private GroundHit sampleGround(Vector3f position, CharacterDescriptor desc) {
        float startOffset = desc.radius() + desc.stepHeight();
        float maxDist = desc.height() * 0.5f + desc.stepHeight() + desc.skinWidth() + 0.5f;
        Vector3f origin = new Vector3f(position.x(), position.y() + startOffset, position.z());
        Optional<RaycastResult> hit = raycastExecutor.raycastClosest(origin, new Vector3f(0f, -1f, 0f), maxDist, desc.collidesWith());
        if (hit.isEmpty()) {
            return GroundHit.none();
        }
        RaycastResult h = hit.get();
        return new GroundHit(true, h.position(), h.normal(), h.material(), h.body());
    }

    private Vector3f tryStepUp(Vector3f from, Vector3f target, CharacterDescriptor desc, float dt) {
        Vector3f move = target.sub(from, new Vector3f());
        Vector3f horizontal = new Vector3f(move.x(), 0f, move.z());
        if (horizontal.length() < 1e-4f) {
            return target;
        }

        Vector3f forward = horizontal.normalize(new Vector3f());
        float probeDist = desc.radius() + desc.skinWidth() + horizontal.length();

        Vector3f footOrigin = new Vector3f(from.x(), from.y() - desc.height() * 0.5f + desc.radius(), from.z());
        Optional<RaycastResult> obstacle = raycastExecutor.raycastClosest(footOrigin, forward, probeDist, desc.collidesWith());
        if (obstacle.isEmpty()) {
            return target;
        }

        Vector3f stepOrigin = new Vector3f(footOrigin).add(new Vector3f(0f, desc.stepHeight(), 0f));
        Optional<RaycastResult> clearance = raycastExecutor.raycastClosest(stepOrigin, forward, probeDist, desc.collidesWith());
        if (clearance.isEmpty()) {
            return new Vector3f(target).add(new Vector3f(0f, desc.stepHeight(), 0f));
        }

        return target;
    }

    private static Vector3f projectOnPlane(Vector3f v, Vector3f n) {
        float dot = v.dot(n);
        return v.sub(new Vector3f(n).mul(dot), new Vector3f());
    }

    private Ode4jCharacterHandle ch(CharacterHandle handle) {
        Ode4jCharacterHandle c = characters.get(handle);
        if (c == null) {
            throw new IllegalArgumentException("Unknown character handle: " + handle);
        }
        return c;
    }

    private record GroundHit(
        boolean hit,
        Vector3f position,
        Vector3f normal,
        PhysicsMaterial material,
        org.dynamisphysics.api.body.RigidBodyHandle body
    ) {
        static GroundHit none() {
            return new GroundHit(false, new Vector3f(), new Vector3f(0f, 1f, 0f), PhysicsMaterial.DEFAULT, null);
        }
    }
}
