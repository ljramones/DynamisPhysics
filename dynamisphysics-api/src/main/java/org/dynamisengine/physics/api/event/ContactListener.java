package org.dynamisengine.physics.api.event;

@FunctionalInterface
public interface ContactListener {
    void onContact(ContactEvent event);
}
