package org.dynamisphysics.api.event;

public sealed interface PhysicsEvent permits
    ContactEvent,
    SeparationEvent,
    TriggerEnterEvent,
    TriggerExitEvent,
    SleepEvent,
    WakeEvent,
    ConstraintBreakEvent,
    FractureEvent,
    SplashEvent,
    WheelSlipEvent,
    FootContactEvent,
    VehicleAirborneEvent {}
