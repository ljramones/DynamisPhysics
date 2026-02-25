package org.dynamisphysics.api.event;

@FunctionalInterface
public interface ContactListener {
    void onContact(ContactEvent event);
}
