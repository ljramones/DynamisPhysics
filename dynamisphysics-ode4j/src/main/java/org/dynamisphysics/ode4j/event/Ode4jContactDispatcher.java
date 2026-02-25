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
    public record DebugContact(Vector3f position, Vector3f normal, float depth) {}
    private record TraceEntry(int order, int bodyIdA, int bodyIdB, int geomIdA, int geomIdB, int contactCount, String pointSig) {}
    private record ManifoldPoint(int index, int px, int py, int pz, int nx, int ny, int nz, int depth) {}

    private final DWorld world;
    private final DJointGroup contactGroup;
    private final Ode4jEventBuffer eventBuffer;
    private final List<PendingPair> pendingPairs = new ArrayList<>(256);
    private final List<DebugContact> debugContacts = new ArrayList<>(256);
    private final DContactBuffer contactBuffer = new DContactBuffer(MAX_CONTACTS);
    private final List<TraceEntry> traceEntries = new ArrayList<>(256);
    private long traceStep = -1L;
    private int traceOrder = 0;

    public final DGeom.DNearCallback callback;

    public Ode4jContactDispatcher(DWorld world, DJointGroup contactGroup, Ode4jEventBuffer eventBuffer) {
        this.world = world;
        this.contactGroup = contactGroup;
        this.eventBuffer = eventBuffer;
        this.callback = this::handleNear;
    }

    public void beginSubstepTrace(long stepIndex) {
        this.traceStep = stepIndex;
        this.traceOrder = 0;
        this.traceEntries.clear();
    }

    private void handleNear(Object data, DGeom o1, DGeom o2) {
        Ode4jBodyHandle h1 = handleFor(o1);
        Ode4jBodyHandle h2 = handleFor(o2);
        int bodyId1 = h1 != null ? h1.bodyId() : Integer.MAX_VALUE;
        int bodyId2 = h2 != null ? h2.bodyId() : Integer.MAX_VALUE;
        int geomId1 = h1 != null ? geomIdFor(o1, h1) : Integer.MAX_VALUE;
        int geomId2 = h2 != null ? geomIdFor(o2, h2) : Integer.MAX_VALUE;
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
            maybeTrace(pair, n);
        }
        maybePrintTrace();
        pendingPairs.clear();
    }

    public void clearQueuedContacts() {
        pendingPairs.clear();
    }

    public List<DebugContact> drainDebugContacts() {
        var copy = List.copyOf(debugContacts);
        debugContacts.clear();
        return copy;
    }

    private void emitContacts(DGeom o1, DGeom o2, DBody bodyA, DBody bodyB, int n) {
        List<ContactPoint> points = new ArrayList<>(n);
        float totalImpulse = 0f;
        Ode4jBodyHandle hA = handleFor(o1);
        Ode4jBodyHandle hB = handleFor(o2);
        int bodyIdA = hA != null ? hA.bodyId() : Integer.MAX_VALUE;
        int bodyIdB = hB != null ? hB.bodyId() : Integer.MAX_VALUE;

        List<ManifoldPoint> manifoldOrder = sortedManifoldOrder(n);
        maybePrintManifold(bodyIdA, bodyIdB, n, manifoldOrder);

        for (ManifoldPoint ordered : manifoldOrder) {
            DContactGeom cg = contactBuffer.getGeomBuffer().get(ordered.index());
            DContact contact = contactBuffer.get(ordered.index());
            contact.geom.set(cg);
            contact.surface.mode = OdeConstants.dContactBounce | OdeConstants.dContactSoftCFM;
            contact.surface.mu = combinedFriction(o1, o2);
            contact.surface.bounce = combinedRestitution(o1, o2);
            contact.surface.soft_cfm = 1e-5;
            DJoint cj = OdeHelper.createContactJoint(world, contactGroup, contact);
            cj.attach(bodyA, bodyB);
            points.add(new ContactPoint(toVec3f(cg.pos), toVec3f(cg.normal), (float) cg.depth, 0f));
            debugContacts.add(new DebugContact(toVec3f(cg.pos), toVec3f(cg.normal), (float) cg.depth));
        }

        RigidBodyHandle handleA = bodyA != null ? (RigidBodyHandle) bodyA.getData() : null;
        RigidBodyHandle handleB = bodyB != null ? (RigidBodyHandle) bodyB.getData() : null;

        eventBuffer.add(new ContactEvent(
            handleA,
            handleB,
            points,
            totalImpulse,
            new Vector3f(),
            materialFor(o1),
            materialFor(o2)
        ));
    }

    private List<ManifoldPoint> sortedManifoldOrder(int n) {
        ArrayList<ManifoldPoint> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            DContactGeom cg = contactBuffer.getGeomBuffer().get(i);
            out.add(new ManifoldPoint(
                i,
                qf((float) cg.pos.get0()),
                qf((float) cg.pos.get1()),
                qf((float) cg.pos.get2()),
                qf((float) cg.normal.get0()),
                qf((float) cg.normal.get1()),
                qf((float) cg.normal.get2()),
                qd((float) cg.depth)
            ));
        }
        out.sort(Comparator
            .comparingInt(ManifoldPoint::px)
            .thenComparingInt(ManifoldPoint::py)
            .thenComparingInt(ManifoldPoint::pz)
            .thenComparingInt(ManifoldPoint::nx)
            .thenComparingInt(ManifoldPoint::ny)
            .thenComparingInt(ManifoldPoint::nz)
            .thenComparingInt(ManifoldPoint::depth)
            .thenComparingInt(ManifoldPoint::index));
        return out;
    }

    private void maybePrintManifold(int bodyIdA, int bodyIdB, int n, List<ManifoldPoint> sorted) {
        if (!traceEnabledForStep(traceStep) || !Boolean.getBoolean("physics.ode4j.trace.manifold")) {
            return;
        }
        int focusBodyId = Integer.getInteger("physics.ode4j.trace.bodyId", Integer.MIN_VALUE);
        if (focusBodyId != Integer.MIN_VALUE && bodyIdA != focusBodyId && bodyIdB != focusBodyId) {
            return;
        }
        String runTag = System.getProperty("physics.ode4j.trace.runTag", "run?");
        StringBuilder before = new StringBuilder();
        for (int i = 0; i < n; i++) {
            if (i > 0) before.append(";");
            DContactGeom cg = contactBuffer.getGeomBuffer().get(i);
            before.append(sig(cg));
        }
        StringBuilder after = new StringBuilder();
        for (int i = 0; i < sorted.size(); i++) {
            if (i > 0) after.append(";");
            DContactGeom cg = contactBuffer.getGeomBuffer().get(sorted.get(i).index());
            after.append(sig(cg));
        }
        System.out.println("TRACE_MANIFOLD runTag=" + runTag
            + " step=" + traceStep
            + " bodyA=" + bodyIdA
            + " bodyB=" + bodyIdB
            + " n=" + n
            + " before=" + before
            + " after=" + after);
    }

    private void maybeTrace(PendingPair pair, int n) {
        if (!traceEnabledForStep(traceStep)) {
            return;
        }
        StringBuilder points = new StringBuilder(64);
        for (int i = 0; i < n; i++) {
            DContactGeom cg = contactBuffer.getGeomBuffer().get(i);
            if (i > 0) {
                points.append("|");
            }
            points.append(q((float) cg.pos.get0())).append(",")
                .append(q((float) cg.pos.get1())).append(",")
                .append(q((float) cg.pos.get2())).append(";")
                .append(q((float) cg.normal.get0())).append(",")
                .append(q((float) cg.normal.get1())).append(",")
                .append(q((float) cg.normal.get2()));
        }
        traceEntries.add(new TraceEntry(
            ++traceOrder,
            pair.bodyIdA(),
            pair.bodyIdB(),
            pair.geomIdA(),
            pair.geomIdB(),
            n,
            points.toString()
        ));
    }

    private void maybePrintTrace() {
        if (!traceEnabledForStep(traceStep)) {
            return;
        }
        String runTag = System.getProperty("physics.ode4j.trace.runTag", "run?");
        System.out.println("TRACE_CONTACT begin runTag=" + runTag + " step=" + traceStep + " entries=" + traceEntries.size());
        for (TraceEntry e : traceEntries) {
            System.out.println("TRACE_CONTACT entry runTag=" + runTag
                + " step=" + traceStep
                + " order=" + e.order()
                + " bodyA=" + e.bodyIdA()
                + " bodyB=" + e.bodyIdB()
                + " geomA=" + e.geomIdA()
                + " geomB=" + e.geomIdB()
                + " n=" + e.contactCount()
                + " points=" + e.pointSig());
        }
        System.out.println("TRACE_CONTACT end runTag=" + runTag + " step=" + traceStep);
    }

    private static boolean traceEnabledForStep(long step) {
        if (!Boolean.getBoolean("physics.ode4j.trace.contacts")) {
            return false;
        }
        int from = Integer.getInteger("physics.ode4j.trace.stepFrom", Integer.MIN_VALUE);
        int to = Integer.getInteger("physics.ode4j.trace.stepTo", Integer.MAX_VALUE);
        return step >= from && step <= to;
    }

    private static String q(float v) {
        return String.format(java.util.Locale.ROOT, "%.4f", v);
    }

    private static int qf(float v) {
        if (v == -0.0f) {
            v = 0.0f;
        }
        return Math.round(v * 100_000f);
    }

    private static int qd(float v) {
        if (v == -0.0f) {
            v = 0.0f;
        }
        return Math.round(v * 1_000_000f);
    }

    private static String sig(DContactGeom cg) {
        return qf((float) cg.pos.get0()) + "," + qf((float) cg.pos.get1()) + "," + qf((float) cg.pos.get2())
            + "|" + qf((float) cg.normal.get0()) + "," + qf((float) cg.normal.get1()) + "," + qf((float) cg.normal.get2())
            + "|" + qd((float) cg.depth);
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
        if (data instanceof Ode4jBodyHandle.GeomRef ref) {
            return ref.handle();
        }
        DBody body = g.getBody();
        if (body != null && body.getData() instanceof Ode4jBodyHandle h) {
            return h;
        }
        return null;
    }

    private static int geomIdFor(DGeom g, Ode4jBodyHandle fallback) {
        Object data = g.getData();
        if (data instanceof Ode4jBodyHandle.GeomRef ref) {
            return ref.geomId();
        }
        return fallback.geomId();
    }
}
