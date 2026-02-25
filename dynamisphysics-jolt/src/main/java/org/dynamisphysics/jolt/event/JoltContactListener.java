package org.dynamisphysics.jolt.event;

import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.ContactManifold;
import com.github.stephengold.joltjni.CustomContactListener;
import com.github.stephengold.joltjni.enumerate.ValidateResult;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.ContactPoint;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.dynamisphysics.jolt.body.JoltBodyRegistry;
import org.vectrix.core.Vector3f;

import java.util.List;

public final class JoltContactListener extends CustomContactListener {
    private final JoltBodyRegistry bodyRegistry;
    private final JoltEventBuffer eventBuffer;

    public JoltContactListener(JoltBodyRegistry bodyRegistry, JoltEventBuffer eventBuffer) {
        this.bodyRegistry = bodyRegistry;
        this.eventBuffer = eventBuffer;
    }

    @Override
    public int onContactValidate(long body1Va, long body2Va, double baseOffsetX, double baseOffsetY, double baseOffsetZ,
        long collisionResultVa) {
        return ValidateResult.AcceptAllContactsForThisBodyPair.ordinal();
    }

    @Override
    public void onContactAdded(long body1Va, long body2Va, long manifoldVa, long settingsVa) {
        emitContact(body1Va, body2Va, manifoldVa);
    }

    @Override
    public void onContactPersisted(long body1Va, long body2Va, long manifoldVa, long settingsVa) {
        emitContact(body1Va, body2Va, manifoldVa);
    }

    @Override
    public void onContactRemoved(long subShapePairVa) {
        // No-op for step 11.
    }

    private void emitContact(long body1Va, long body2Va, long manifoldVa) {
        Body bodyA = new Body(body1Va);
        Body bodyB = new Body(body2Va);
        JoltBodyHandle hA = bodyRegistry.getByJoltId(bodyA.getId());
        JoltBodyHandle hB = bodyRegistry.getByJoltId(bodyB.getId());
        if (hA == null && hB == null) {
            return;
        }

        int stableA = hA != null ? hA.bodyId() : Integer.MAX_VALUE;
        int stableB = hB != null ? hB.bodyId() : Integer.MAX_VALUE;
        JoltBodyHandle first = stableA <= stableB ? hA : hB;
        JoltBodyHandle second = stableA <= stableB ? hB : hA;

        ContactManifold manifold = new ContactManifold(manifoldVa);
        Vector3f normal = new Vector3f(manifold.getWorldSpaceNormal().getX(), manifold.getWorldSpaceNormal().getY(),
            manifold.getWorldSpaceNormal().getZ());
        Vector3f point = new Vector3f((float) manifold.getBaseOffset().xx(), (float) manifold.getBaseOffset().yy(),
            (float) manifold.getBaseOffset().zz());
        float depth = manifold.getPenetrationDepth();

        eventBuffer.add(new ContactEvent(
            first,
            second,
            List.of(new ContactPoint(point, normal, depth, 0f)),
            0f,
            new Vector3f(),
            first != null ? first.config().material() : PhysicsMaterial.DEFAULT,
            second != null ? second.config().material() : PhysicsMaterial.DEFAULT
        ));
    }
}
