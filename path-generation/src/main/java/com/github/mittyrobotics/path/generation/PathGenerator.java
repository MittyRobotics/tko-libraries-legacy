package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocity;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;
import com.github.mittyrobotics.path.generation.splines.CubicHermiteSpline;
import com.github.mittyrobotics.path.generation.splines.QuinticHermiteSpline;

public class PathGenerator {
    private static PathGenerator instance;

    public static PathGenerator getInstance() {
        return instance;
    }

    public Parametric[] generateCubicHermiteSplinePath(TransformWithVelocity[] waypoints){
        Parametric[] parametrics = new Parametric[waypoints.length-1];
        for(int i = 0; i < parametrics.length; i++){
            parametrics[i] = new CubicHermiteSpline(waypoints[i], waypoints[i+1]);
        }
        return parametrics;
    }

    public Parametric[] generateQuinticHermiteSplinePath(TransformWithVelocityAndCurvature[] waypoints){
        Parametric[] parametrics = new Parametric[waypoints.length-1];
        for(int i = 0; i < parametrics.length; i++){
            parametrics[i] = new QuinticHermiteSpline(waypoints[i], waypoints[i+1]);
        }
        return parametrics;
    }

}
