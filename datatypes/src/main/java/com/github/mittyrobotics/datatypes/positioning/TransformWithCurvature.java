package com.github.mittyrobotics.datatypes.positioning;

import com.github.mittyrobotics.datatypes.interfaces.WithCurvature;

public class TransformWithCurvature extends Transform implements WithCurvature {
    private final double curvature;

    public TransformWithCurvature(Transform transform, double curvature) {
        super(transform);
        this.curvature = curvature;
    }

    @Override
    public double getCurvature() {
        return curvature;
    }
}
