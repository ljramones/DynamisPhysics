package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

import java.util.List;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toOde;

public final class Ode4jConstraintFactory {
    private Ode4jConstraintFactory() {}

    public static Ode4jConstraintHandle create(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        Ode4jBodyRegistry bodyRegistry
    ) {
        DBody bodyA = resolveBody(desc.bodyA(), bodyRegistry);
        DBody bodyB = resolveBody(desc.bodyB(), bodyRegistry);

        return switch (desc.type()) {
            case FIXED -> createFixed(constraintId, desc, world, bodyA, bodyB);
            case BALL_SOCKET -> createBallSocket(constraintId, desc, world, bodyA, bodyB);
            case HINGE -> createHinge(constraintId, desc, world, bodyA, bodyB);
            case HINGE2 -> createHinge2(constraintId, desc, world, bodyA, bodyB);
            case SLIDER -> createSlider(constraintId, desc, world, bodyA, bodyB);
            case CONE_TWIST -> createConeTwist(constraintId, desc, world, bodyA, bodyB);
            case SIX_DOF -> createSixDof(constraintId, desc, world, bodyA, bodyB, false);
            case SIX_DOF_SPRING -> createSixDof(constraintId, desc, world, bodyA, bodyB, true);
            case GEAR -> createGear(constraintId, desc, world, bodyA, bodyB);
            case RACK_PINION -> createRackPinion(constraintId, desc, world, bodyA, bodyB);
            case PULLEY -> createPulley(constraintId, desc, world, bodyA, bodyB);
        };
    }

    private static Ode4jConstraintHandle createFixed(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DFixedJoint j = OdeHelper.createFixedJoint(world);
        j.attach(bodyA, bodyB);
        j.setFixed();
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createBallSocket(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DBallJoint j = OdeHelper.createBallJoint(world);
        j.attach(bodyA, bodyB);
        j.setAnchor(toOde(desc.pivotA()));
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createHinge(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DHingeJoint j = OdeHelper.createHingeJoint(world);
        j.attach(bodyA, bodyB);
        j.setAnchor(toOde(desc.pivotA()));
        j.setAxis(toOde(desc.axisA()));
        applyHingeLimits(j, desc.limits());
        applyHingeMotor(j, desc.motor());
        wakeBodies(bodyA, bodyB);
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createHinge2(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DHinge2Joint j = OdeHelper.createHinge2Joint(world);
        j.attach(bodyA, bodyB);
        j.setAnchor(toOde(desc.pivotA()));
        j.setAxes(toOde(desc.axisA()), toOde(desc.axisB()));
        j.setParamLoStop(desc.limits().angularLowerLimit());
        j.setParamHiStop(desc.limits().angularUpperLimit());
        applyHinge2Motor(j, desc.motor());
        wakeBodies(bodyA, bodyB);
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createSlider(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DSliderJoint j = OdeHelper.createSliderJoint(world);
        j.attach(bodyA, bodyB);
        j.setAxis(toOde(desc.axisA()));
        j.setParamLoStop(desc.limits().linearLowerLimit());
        j.setParamHiStop(desc.limits().linearUpperLimit());
        applySliderMotor(j, desc.motor());
        wakeBodies(bodyA, bodyB);
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createConeTwist(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DAMotorJoint j = OdeHelper.createAMotorJoint(world);
        j.attach(bodyA, bodyB);
        j.setMode(DAMotorJoint.AMotorMode.dAMotorEuler);
        j.setNumAxes(3);
        j.setAxis(0, 1, toOde(desc.axisA()));
        j.setAxis(2, 2, toOde(desc.axisB()));
        j.setParamLoStop(desc.limits().angularLowerLimit());
        j.setParamHiStop(desc.limits().angularUpperLimit());
        j.setParamLoStop2(desc.limits().angularLowerLimit());
        j.setParamHiStop2(desc.limits().angularUpperLimit());
        j.setParamLoStop3(-desc.limits().angularUpperLimit());
        j.setParamHiStop3(desc.limits().angularUpperLimit());
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createSixDof(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB,
        boolean spring
    ) {
        DAMotorJoint amotor = OdeHelper.createAMotorJoint(world);
        amotor.attach(bodyA, bodyB);
        amotor.setMode(DAMotorJoint.AMotorMode.dAMotorEuler);
        amotor.setNumAxes(3);
        amotor.setAxis(0, 1, toOde(desc.axisA()));
        amotor.setAxis(2, 2, toOde(desc.axisB()));
        amotor.setParamLoStop(desc.limits().angularLowerLimit());
        amotor.setParamHiStop(desc.limits().angularUpperLimit());
        if (spring) {
            amotor.setParam(org.ode4j.ode.DJoint.PARAM_N.dParamCFM1, 1e-3);
        }

        DSliderJoint slider = OdeHelper.createSliderJoint(world);
        slider.attach(bodyA, bodyB);
        slider.setAxis(toOde(desc.axisA()));
        slider.setParamLoStop(desc.limits().linearLowerLimit());
        slider.setParamHiStop(desc.limits().linearUpperLimit());
        enableFeedback(amotor, desc);
        enableFeedback(slider, desc);
        return new Ode4jConstraintHandle(constraintId, desc, List.of(amotor, slider));
    }

    private static Ode4jConstraintHandle createGear(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        // Placeholder joint for Step 5; true gear torque coupling lands in later tuning.
        DFixedJoint j = OdeHelper.createFixedJoint(world);
        j.attach(bodyA, bodyB);
        j.setFixed();
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createRackPinion(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DSliderJoint j = OdeHelper.createSliderJoint(world);
        j.attach(bodyA, bodyB);
        j.setAxis(toOde(desc.axisA()));
        enableFeedback(j, desc);
        return new Ode4jConstraintHandle(constraintId, desc, j);
    }

    private static Ode4jConstraintHandle createPulley(
        int constraintId,
        ConstraintDesc desc,
        DWorld world,
        DBody bodyA,
        DBody bodyB
    ) {
        DSliderJoint j1 = OdeHelper.createSliderJoint(world);
        j1.attach(bodyA, null);
        j1.setAxis(toOde(desc.axisA()));
        j1.setParamLoStop(desc.limits().linearLowerLimit());
        j1.setParamHiStop(desc.limits().linearUpperLimit());

        DSliderJoint j2 = OdeHelper.createSliderJoint(world);
        j2.attach(bodyB, null);
        j2.setAxis(toOde(desc.axisA()));
        j2.setParamLoStop(desc.limits().linearLowerLimit());
        j2.setParamHiStop(desc.limits().linearUpperLimit());
        enableFeedback(j1, desc);
        enableFeedback(j2, desc);
        return new Ode4jConstraintHandle(constraintId, desc, List.of(j1, j2));
    }

    private static DBody resolveBody(RigidBodyHandle h, Ode4jBodyRegistry reg) {
        if (h == null) {
            return null;
        }
        Ode4jBodyHandle oh = reg.getHandle(h);
        return oh != null ? oh.body() : null;
    }

    private static void enableFeedback(DJoint joint, ConstraintDesc desc) {
        if (desc.breakForce() > 0f || desc.breakTorque() > 0f) {
            joint.setFeedback(new DJoint.DJointFeedback());
        }
    }

    private static void applyHingeLimits(DHingeJoint j, ConstraintLimits lim) {
        if (lim.angularLowerLimit() != -Float.MAX_VALUE) {
            j.setParamLoStop(lim.angularLowerLimit());
        }
        if (lim.angularUpperLimit() != Float.MAX_VALUE) {
            j.setParamHiStop(lim.angularUpperLimit());
        }
    }

    private static void applyHingeMotor(DHingeJoint j, ConstraintMotor m) {
        if (!m.enabled()) {
            return;
        }
        j.setParamVel(m.targetVelocity());
        j.setParamFMax(m.maxForce());
    }

    private static void applyHinge2Motor(DHinge2Joint j, ConstraintMotor m) {
        if (!m.enabled()) {
            return;
        }
        j.setParamVel2(m.targetVelocity());
        j.setParamFMax2(m.maxForce());
    }

    private static void applySliderMotor(DSliderJoint j, ConstraintMotor m) {
        if (!m.enabled()) {
            return;
        }
        j.setParamVel(m.targetVelocity());
        j.setParamFMax(m.maxForce());
    }

    private static void wakeBodies(DBody bodyA, DBody bodyB) {
        if (bodyA != null) {
            bodyA.enable();
        }
        if (bodyB != null) {
            bodyB.enable();
        }
    }
}
