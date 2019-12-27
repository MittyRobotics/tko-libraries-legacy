package com.github.mittyrobotics.datatypes.positioning;

public class TransformWithVelocityAndCurvature extends TransformWithVelocity {
    private double curvature;

    public TransformWithVelocityAndCurvature(Transform transform, double velocity, double curvature) {
        super(transform, velocity);
        this.curvature = curvature;
    }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }
}
