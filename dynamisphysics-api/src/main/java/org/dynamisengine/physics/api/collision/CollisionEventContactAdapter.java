package org.dynamisengine.physics.api.collision;

import org.dynamisengine.collision.events.CollisionEvent;
import org.dynamisengine.collision.events.CollisionEventType;

import java.util.ArrayList;
import java.util.List;

/**
 * Transitional adapter for legacy collision event flows.
 *
 * <p>This keeps compatibility with collision event inputs while routing Physics resolution through
 * Physics-owned seam types.
 */
public final class CollisionEventContactAdapter {

    private CollisionEventContactAdapter() {
    }

    public static <T> List<DetectedCollisionContact<T>> toDetectedContacts(Iterable<CollisionEvent<T>> events) {
        if (events == null) {
            throw new IllegalArgumentException("events must not be null");
        }
        List<DetectedCollisionContact<T>> out = new ArrayList<>();
        for (CollisionEvent<T> event : events) {
            if (event == null
                    || !event.responseEnabled()
                    || event.type() == CollisionEventType.EXIT
                    || event.manifold() == null) {
                continue;
            }
            out.add(new DetectedCollisionContact<>(
                    event.pair().first(),
                    event.pair().second(),
                    event.manifold()));
        }
        return List.copyOf(out);
    }
}
