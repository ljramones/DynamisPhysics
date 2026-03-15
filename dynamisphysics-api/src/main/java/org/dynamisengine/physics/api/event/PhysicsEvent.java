package org.dynamisengine.physics.api.event;

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
