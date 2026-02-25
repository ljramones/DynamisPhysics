package org.dynamisphysics.ode4j.constraint;

import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.event.ConstraintBreakEvent;
import org.dynamisphysics.ode4j.body.Ode4jBodyRegistry;
import org.dynamisphysics.ode4j.event.Ode4jEventBuffer;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class Ode4jConstraintRegistry {
    private final DWorld world;
    private final Ode4jBodyRegistry bodyRegistry;
    private final Ode4jEventBuffer eventBuffer;
    private final Map<ConstraintHandle, Ode4jConstraintHandle> handlesByHandle = new LinkedHashMap<>();
    private final Map<Integer, Ode4jConstraintHandle> handlesById = new LinkedHashMap<>();
    private final Map<Integer, Ode4jConstraintHandle> lookupById = new HashMap<>();
    private int nextConstraintId = 1;

    public Ode4jConstraintRegistry(DWorld world, Ode4jBodyRegistry bodyRegistry, Ode4jEventBuffer eventBuffer) {
        this.world = world;
        this.bodyRegistry = bodyRegistry;
        this.eventBuffer = eventBuffer;
    }

    public ConstraintHandle add(ConstraintDesc desc) {
        return addWithId(desc, nextConstraintId++);
    }

    public ConstraintHandle addWithId(ConstraintDesc desc, int constraintId) {
        nextConstraintId = Math.max(nextConstraintId, constraintId + 1);
        Ode4jConstraintHandle h = Ode4jConstraintFactory.create(constraintId, desc, world, bodyRegistry);
        handlesByHandle.put(h, h);
        handlesById.put(h.constraintId(), h);
        lookupById.put(h.constraintId(), h);
        return h;
    }

    public void remove(ConstraintHandle h) {
        Ode4jConstraintHandle oh = handlesByHandle.remove(h);
        if (oh != null) {
            handlesById.remove(oh.constraintId());
            lookupById.remove(oh.constraintId());
            oh.kill();
        }
    }

    public void setEnabled(ConstraintHandle h, boolean enabled) {
        Ode4jConstraintHandle oh = handlesByHandle.get(h);
        if (oh == null) {
            return;
        }
        oh.allJoints().forEach(j -> {
            if (enabled) {
                j.enable();
            } else {
                j.disable();
            }
        });
    }

    public void setMotorTarget(ConstraintHandle h, float target) {
        Ode4jConstraintHandle oh = handlesByHandle.get(h);
        if (oh == null || !oh.desc().motor().enabled()) {
            return;
        }
        oh.allJoints().forEach(j -> {
            if (j instanceof DHingeJoint hj) {
                hj.setParamVel(target);
            }
            if (j instanceof DSliderJoint sj) {
                sj.setParamVel(target);
            }
            if (j instanceof DHinge2Joint h2) {
                h2.setParamVel2(target);
            }
        });
    }

    public void checkBreakForces() {
        var toRemove = new ArrayList<ConstraintHandle>();

        for (Ode4jConstraintHandle oh : handlesByHandle.values()) {
            ConstraintDesc desc = oh.desc();
            if (desc.breakForce() <= 0f && desc.breakTorque() <= 0f) {
                continue;
            }
            for (DJoint joint : oh.allJoints()) {
                DJoint.DJointFeedback fb = joint.getFeedback();
                float forceA = 0f;
                float torqueA = 0f;
                if (fb != null) {
                    forceA = toVec3f(fb.f1).length();
                    torqueA = toVec3f(fb.t1).length();
                }
                // Some joint combinations report little/no feedback force in this stage.
                // Fallback to relative velocity magnitude as a break proxy.
                if (forceA <= 1e-6f) {
                    forceA = approximateRelativeForce(joint);
                }
                boolean broken =
                    (desc.breakForce() > 0f && forceA > desc.breakForce())
                        || (desc.breakTorque() > 0f && torqueA > desc.breakTorque());

                if (broken) {
                    eventBuffer.add(new ConstraintBreakEvent(oh, forceA));
                    toRemove.add(oh);
                    break;
                }
            }
        }

        toRemove.forEach(this::remove);
    }

    public int constraintCount() {
        return handlesByHandle.size();
    }

    public Ode4jConstraintHandle getHandleById(int id) {
        return lookupById.get(id);
    }

    public List<Ode4jConstraintHandle> constraintsInIdOrder() {
        return handlesById.values().stream()
            .sorted(Comparator.comparingInt(Ode4jConstraintHandle::constraintId))
            .toList();
    }

    public void clearAllConstraints() {
        var snapshot = List.copyOf(handlesByHandle.keySet());
        for (ConstraintHandle h : snapshot) {
            remove(h);
        }
    }

    public int nextConstraintId() {
        return nextConstraintId;
    }

    private static Vector3f toVec3f(DVector3C v) {
        return new Vector3f((float) v.get0(), (float) v.get1(), (float) v.get2());
    }

    private static float approximateRelativeForce(DJoint joint) {
        var b0 = joint.getBody(0);
        var b1 = joint.getBody(1);
        if (b0 == null && b1 == null) {
            return 0f;
        }
        if (b0 == null) {
            return toVec3f(b1.getLinearVel()).length() * 100f;
        }
        if (b1 == null) {
            return toVec3f(b0.getLinearVel()).length() * 100f;
        }
        Vector3f v0 = toVec3f(b0.getLinearVel());
        Vector3f v1 = toVec3f(b1.getLinearVel());
        return v0.sub(v1).length() * 100f;
    }
}
