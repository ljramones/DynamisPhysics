package org.dynamisphysics.ode4j.event;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.ContactPoint;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import static org.dynamisphysics.ode4j.world.Ode4jConversions.toVec3f;

public final class Ode4jContactDispatcher {
    private static final int MAX_CONTACTS = 8;
    private record PendingPair(DGeom o1, DGeom o2, int bodyIdA, int bodyIdB, int geomIdA, int geomIdB) {}

    private final DWorld world;
    private final DJointGroup contactGroup;
    private final Ode4jEventBuffer eventBuffer;
    private final List<PendingPair> pendingPairs = new ArrayList<>(256);
    private final DContactBuffer contactBuffer = new DContactBuffer(MAX_CONTACTS);

    public final DGeom.DNearCallback callback;

    public Ode4jContactDispatcher(DWorld world, DJointGroup contactGroup, Ode4jEventBuffer eventBuffer) {
        this.world = world;
        this.contactGroup = contactGroup;
        this.eventBuffer = eventBuffer;
        this.callback = this::handleNear;
    }

    private void handleNear(Object data, DGeom o1, DGeom o2) {
        Ode4jBodyHandle h1 = handleFor(o1);
        Ode4jBodyHandle h2 = handleFor(o2);
        int bodyId1 = h1 != null ? h1.bodyId() : Integer.MAX_VALUE;
        int bodyId2 = h2 != null ? h2.bodyId() : Integer.MAX_VALUE;
        int geomId1 = h1 != null ? h1.geomId() : Integer.MAX_VALUE;
        int geomId2 = h2 != null ? h2.geomId() : Integer.MAX_VALUE;
        if (bodyId1 > bodyId2 || (bodyId1 == bodyId2 && geomId1 > geomId2)) {
            DGeom gtmp = o1;
            o1 = o2;
            o2 = gtmp;
            Ode4jBodyHandle htmp = h1;
            h1 = h2;
            h2 = htmp;
        }
        DBody bodyA = o1.getBody();
        DBody bodyB = o2.getBody();
        if (bodyA == null && bodyB == null) {
            return;
        }
        pendingPairs.add(new PendingPair(o1, o2, bodyId1, bodyId2, geomId1, geomId2));
    }

    public void resolveQueuedContacts() {
        pendingPairs.sort(Comparator
            .comparingInt(PendingPair::bodyIdA)
            .thenComparingInt(PendingPair::bodyIdB)
            .thenComparingInt(PendingPair::geomIdA)
            .thenComparingInt(PendingPair::geomIdB));

        for (PendingPair pair : pendingPairs) {
            DGeom o1 = pair.o1();
            DGeom o2 = pair.o2();
            int n = OdeHelper.collide(o1, o2, MAX_CONTACTS, contactBuffer.getGeomBuffer());
            if (n == 0) {
                continue;
            }
            DBody bodyA = o1.getBody();
            DBody bodyB = o2.getBody();
            if (bodyA == null && bodyB == null) {
                continue;
            }

            emitContacts(o1, o2, bodyA, bodyB, n);
        }
        pendingPairs.clear();
    }

    public void clearQueuedContacts() {
        pendingPairs.clear();
    }

    private void emitContacts(DGeom o1, DGeom o2, DBody bodyA, DBody bodyB, int n) {
        List<ContactPoint> points = new ArrayList<>(n);
        float totalImpulse = 0f;

        for (int i = 0; i < n; i++) {
            DContactGeom cg = contactBuffer.getGeomBuffer().get(i);
            DContact contact = contactBuffer.get(i);
            contact.geom.set(cg);
            contact.surface.mode = OdeConstants.dContactBounce | OdeConstants.dContactSoftCFM;
            contact.surface.mu = combinedFriction(o1, o2);
            contact.surface.bounce = combinedRestitution(o1, o2);
            contact.surface.soft_cfm = 1e-5;
            DJoint cj = OdeHelper.createContactJoint(world, contactGroup, contact);
            cj.attach(bodyA, bodyB);
            points.add(new ContactPoint(toVec3f(cg.pos), toVec3f(cg.normal), (float) cg.depth, 0f));
        }

        RigidBodyHandle hA = bodyA != null ? (RigidBodyHandle) bodyA.getData() : null;
        RigidBodyHandle hB = bodyB != null ? (RigidBodyHandle) bodyB.getData() : null;

        eventBuffer.add(new ContactEvent(
            hA,
            hB,
            points,
            totalImpulse,
            new Vector3f(),
            materialFor(o1),
            materialFor(o2)
        ));
    }

    private static float combinedFriction(DGeom a, DGeom b) {
        return Math.min(materialFor(a).friction(), materialFor(b).friction());
    }

    private static float combinedRestitution(DGeom a, DGeom b) {
        return (materialFor(a).restitution() + materialFor(b).restitution()) / 2f;
    }

    private static PhysicsMaterial materialFor(DGeom g) {
        Ode4jBodyHandle h = handleFor(g);
        if (h != null) {
            return h.config().material();
        }
        return PhysicsMaterial.DEFAULT;
    }

    private static Ode4jBodyHandle handleFor(DGeom g) {
        Object data = g.getData();
        if (data instanceof Ode4jBodyHandle h) {
            return h;
        }
        DBody body = g.getBody();
        if (body != null && body.getData() instanceof Ode4jBodyHandle h) {
            return h;
        }
        return null;
    }
}
