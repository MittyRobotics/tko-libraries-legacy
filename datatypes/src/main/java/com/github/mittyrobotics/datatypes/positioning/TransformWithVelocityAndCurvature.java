package com.github.mittyrobotics.datatypes.positioning;

import com.github.mittyrobotics.datatypes.interfaces.WithCurvature;
import com.github.mittyrobotics.datatypes.interfaces.WithVelocity;

public class TransformWithVelocityAndCurvature extends Transform implements WithVelocity, WithCurvature {
    private double velocity;
    private double curvature;


    public TransformWithVelocityAndCurvature(Transform transform) {
        super(transform);
        this.curvature = 0;
        this.velocity = 0;
    }

    public TransformWithVelocityAndCurvature(Transform transform, double velocity, double curvature) {
        super(transform);
        this.curvature = curvature;
        this.velocity = velocity;
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public double getCurvature() {
        return curvature;
    }
}
