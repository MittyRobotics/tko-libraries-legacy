/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocity;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;
import com.github.mittyrobotics.path.generation.splines.CubicHermiteSpline;
import com.github.mittyrobotics.path.generation.splines.QuinticHermiteSpline;

public class PathGenerator {

    public static Parametric[] generateCubicHermiteSplinePath(TransformWithVelocity[] waypoints) {
        Parametric[] parametrics = new Parametric[waypoints.length - 1];
        for (int i = 0; i < parametrics.length; i++) {
            parametrics[i] = new CubicHermiteSpline(waypoints[i], waypoints[i + 1]);
        }
        return parametrics;
    }

    public static Parametric[] generateQuinticHermiteSplinePath(TransformWithVelocityAndCurvature[] waypoints) {
        Parametric[] parametrics = new Parametric[waypoints.length - 1];

        for (int i = 0; i < parametrics.length; i++) {
            parametrics[i] = new QuinticHermiteSpline(waypoints[i], waypoints[i + 1]);
        }
        return parametrics;
    }

    public static Parametric[] generateQuinticHermiteSplinePath(Transform[] waypoints) {
        Parametric[] parametrics = new Parametric[waypoints.length - 1];

        for (int i = 0; i < parametrics.length; i++) {
            parametrics[i] = new QuinticHermiteSpline(waypoints[i], waypoints[i + 1]);
        }
        return parametrics;
    }
}
