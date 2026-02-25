package org.dynamisphysics.api;

public record PacejkaCoeffs(
    float stiffness,
    float shape,
    float peak,
    float curvature
) {
    public static PacejkaCoeffs defaultLongitudinal() {
        return new PacejkaCoeffs(10f, 1.9f, 1.0f, 0.97f);
    }

    public static PacejkaCoeffs defaultLateral() {
        return new PacejkaCoeffs(6f, 1.3f, 1.0f, -0.2f);
    }

    public PacejkaCoeffs withPeak(float newPeak) {
        return new PacejkaCoeffs(stiffness, shape, newPeak, curvature);
    }
}
