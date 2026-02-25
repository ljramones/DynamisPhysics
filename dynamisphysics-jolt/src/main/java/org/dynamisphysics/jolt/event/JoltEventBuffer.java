package org.dynamisphysics.jolt.event;

import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.jolt.body.JoltBodyHandle;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class JoltEventBuffer {
    private final List<PhysicsEvent> buffer = new ArrayList<>(256);

    public void add(PhysicsEvent event) {
        synchronized (buffer) {
            buffer.add(event);
        }
    }

    public List<PhysicsEvent> drain() {
        synchronized (buffer) {
            if (buffer.isEmpty()) {
                return List.of();
            }
            List<PhysicsEvent> copy = new ArrayList<>(buffer);
            copy.sort(EVENT_ORDER);
            buffer.clear();
            return List.copyOf(copy);
        }
    }

    public void clear() {
        synchronized (buffer) {
            buffer.clear();
        }
    }

    private static final Comparator<PhysicsEvent> EVENT_ORDER = (a, b) -> {
        int byType = a.getClass().getName().compareTo(b.getClass().getName());
        if (byType != 0) {
            return byType;
        }
        if (a instanceof ContactEvent ca && b instanceof ContactEvent cb) {
            int cmpA = Integer.compare(bodyId(ca.bodyA()), bodyId(cb.bodyA()));
            if (cmpA != 0) {
                return cmpA;
            }
            int cmpB = Integer.compare(bodyId(ca.bodyB()), bodyId(cb.bodyB()));
            if (cmpB != 0) {
                return cmpB;
            }
            return Integer.compare(ca.points().size(), cb.points().size());
        }
        return 0;
    };

    private static int bodyId(RigidBodyHandle handle) {
        return handle instanceof JoltBodyHandle jh ? jh.bodyId() : Integer.MAX_VALUE;
    }
}
