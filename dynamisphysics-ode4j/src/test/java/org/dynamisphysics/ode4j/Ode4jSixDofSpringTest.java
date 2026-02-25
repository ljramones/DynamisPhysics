package org.dynamisphysics.ode4j;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jSixDofSpringTest {
    private PhysicsWorld world;

    @BeforeAll
    static void registerBackend() {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    @BeforeEach
    void setUp() {
        world = PhysicsWorldFactory.create(new PhysicsWorldConfig(
            PhysicsBackend.ODE4J,
            new Vector3f(0f, 0f, 0f),
            1f / 60f,
            1,
            20,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true
        ));
    }

    @AfterEach
    void tearDown() {
        world.destroy();
    }

    @Test
    void linearSpringOscillationDecays() {
        RigidBodyHandle anchor = spawnStatic(0f, 0f, 0f);
        RigidBodyHandle body = spawnDynamic(0f, 2f, 0f);

        world.addConstraint(new ConstraintDesc(
            ConstraintType.SIX_DOF_SPRING,
            anchor,
            body,
            new Vector3f(),
            new Vector3f(),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(1f, 0f, 0f),
            new ConstraintLimits(-10f, 10f, -1f, 1f),
            // k=120, c=4 (underdamped): should oscillate and decay.
            new ConstraintMotor(true, 0f, 0f, 120f, 4f),
            0f,
            0f
        ));
        float restY = world.getBodyState(body).position().y();
        world.applyImpulse(body, new Vector3f(0f, 6f, 0f), new Vector3f());

        List<Float> ys = new ArrayList<>();
        for (int i = 0; i < 600; i++) {
            world.step(1f / 60f, 1);
            ys.add(world.getBodyState(body).position().y() - restY);
        }

        boolean crossedRest = false;
        for (int i = 1; i < ys.size(); i++) {
            if ((ys.get(i - 1) > 0f && ys.get(i) < 0f) || (ys.get(i - 1) < 0f && ys.get(i) > 0f)) {
                crossedRest = true;
                break;
            }
        }
        assertTrue(crossedRest, "Expected underdamped oscillation to cross rest position");

        float firstWindowPeak = 0f;
        float lastWindowPeak = 0f;
        for (int i = 0; i < 200; i++) {
            firstWindowPeak = java.lang.Math.max(firstWindowPeak, java.lang.Math.abs(ys.get(i)));
        }
        for (int i = ys.size() - 200; i < ys.size(); i++) {
            lastWindowPeak = java.lang.Math.max(lastWindowPeak, java.lang.Math.abs(ys.get(i)));
        }
        assertTrue(lastWindowPeak < firstWindowPeak * 0.6f,
            "Expected oscillation envelope decay; firstPeak=" + firstWindowPeak + " lastPeak=" + lastWindowPeak);
        assertTrue(java.lang.Math.abs(ys.get(ys.size() - 1)) < 0.25f);
    }

    @Test
    void criticalDampingConvergesWithoutLargeOvershoot() {
        RigidBodyHandle anchor = spawnStatic(0f, 0f, 0f);
        RigidBodyHandle body = spawnDynamic(0f, 2f, 0f);

        world.addConstraint(new ConstraintDesc(
            ConstraintType.SIX_DOF_SPRING,
            anchor,
            body,
            new Vector3f(),
            new Vector3f(),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(1f, 0f, 0f),
            new ConstraintLimits(-10f, 10f, -1f, 1f),
            // k=120, c~22 (near critical for m=1)
            new ConstraintMotor(true, 0f, 0f, 120f, 22f),
            0f,
            0f
        ));
        float restY = world.getBodyState(body).position().y();
        world.applyImpulse(body, new Vector3f(0f, 6f, 0f), new Vector3f());

        float minY = Float.MAX_VALUE;
        for (int i = 0; i < 360; i++) {
            world.step(1f / 60f, 1);
            BodyState state = world.getBodyState(body);
            minY = java.lang.Math.min(minY, state.position().y() - restY);
        }

        BodyState finalState = world.getBodyState(body);
        assertTrue(minY > -0.1f, "Expected near-critical damping to avoid large overshoot; minY=" + minY);
        assertTrue(java.lang.Math.abs(finalState.position().y() - restY) < 0.2f,
            "Expected convergence near rest; y=" + finalState.position().y() + " restY=" + restY);
        assertTrue(java.lang.Math.abs(finalState.linearVelocity().y()) < 0.4f,
            "Expected low residual velocity; vy=" + finalState.linearVelocity().y());
    }

    private RigidBodyHandle spawnDynamic(float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }

    private RigidBodyHandle spawnStatic(float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.5f, 0.5f, 0.5f), 0f)
            .mode(BodyMode.STATIC)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }
}
