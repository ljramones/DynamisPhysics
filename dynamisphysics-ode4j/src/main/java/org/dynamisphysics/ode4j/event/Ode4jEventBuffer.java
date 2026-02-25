package org.dynamisphysics.ode4j.event;

import org.dynamisphysics.api.event.PhysicsEvent;

import java.util.ArrayList;
import java.util.List;

public final class Ode4jEventBuffer {
    private final List<PhysicsEvent> buffer = new ArrayList<>(256);

    public void add(PhysicsEvent event) { buffer.add(event); }
    public List<PhysicsEvent> drain() {
        var copy = List.copyOf(buffer);
        buffer.clear();
        return copy;
    }
    public void clear() { buffer.clear(); }
    public int size() { return buffer.size(); }
}
