package org.dynamisphysics.api.material;

public record PhysicsMaterial(
    float friction,
    float restitution,
    float rollingFriction,
    float spinningFriction,
    String tag
) {
    public static final PhysicsMaterial GRASS = new PhysicsMaterial(0.6f, 0.4f, 0.10f, 0.05f, "grass");
    public static final PhysicsMaterial ROCK = new PhysicsMaterial(0.9f, 0.1f, 0.05f, 0.02f, "rock");
    public static final PhysicsMaterial DIRT = new PhysicsMaterial(0.7f, 0.3f, 0.15f, 0.08f, "dirt");
    public static final PhysicsMaterial SAND = new PhysicsMaterial(0.5f, 0.2f, 0.30f, 0.20f, "sand");
    public static final PhysicsMaterial SNOW = new PhysicsMaterial(0.3f, 0.1f, 0.20f, 0.10f, "snow");
    public static final PhysicsMaterial MUD = new PhysicsMaterial(0.4f, 0.2f, 0.40f, 0.30f, "mud");
    public static final PhysicsMaterial ICE = new PhysicsMaterial(0.05f, 0.1f, 0.02f, 0.01f, "ice");
    public static final PhysicsMaterial ASPHALT = new PhysicsMaterial(0.8f, 0.2f, 0.05f, 0.02f, "asphalt");
    public static final PhysicsMaterial WOOD = new PhysicsMaterial(0.6f, 0.5f, 0.10f, 0.05f, "wood");
    public static final PhysicsMaterial METAL = new PhysicsMaterial(0.5f, 0.4f, 0.02f, 0.01f, "metal");
    public static final PhysicsMaterial RUBBER = new PhysicsMaterial(0.9f, 0.8f, 0.10f, 0.05f, "rubber");
    public static final PhysicsMaterial WATER = new PhysicsMaterial(0.1f, 0.0f, 0.00f, 0.00f, "water");
    public static final PhysicsMaterial DEFAULT = new PhysicsMaterial(0.5f, 0.3f, 0.05f, 0.02f, "default");
    public static final PhysicsMaterial NULL = new PhysicsMaterial(0.0f, 0.0f, 0.00f, 0.00f, "");

    public PhysicsMaterial combine(PhysicsMaterial other, MaterialCombineMode mode) {
        return switch (mode) {
            case AVERAGE -> new PhysicsMaterial(
                (friction + other.friction) / 2f,
                (restitution + other.restitution) / 2f,
                (rollingFriction + other.rollingFriction) / 2f,
                (spinningFriction + other.spinningFriction) / 2f,
                tag
            );
            case MULTIPLY -> new PhysicsMaterial(
                friction * other.friction,
                restitution * other.restitution,
                rollingFriction * other.rollingFriction,
                spinningFriction * other.spinningFriction,
                tag
            );
            case MIN -> new PhysicsMaterial(
                Math.min(friction, other.friction),
                Math.min(restitution, other.restitution),
                Math.min(rollingFriction, other.rollingFriction),
                Math.min(spinningFriction, other.spinningFriction),
                tag
            );
            case MAX -> new PhysicsMaterial(
                Math.max(friction, other.friction),
                Math.max(restitution, other.restitution),
                Math.max(rollingFriction, other.rollingFriction),
                Math.max(spinningFriction, other.spinningFriction),
                tag
            );
        };
    }
}
