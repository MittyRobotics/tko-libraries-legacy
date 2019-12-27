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

package com.github.mittyrobotics.path.generation.paths;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocity;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;
import com.github.mittyrobotics.path.generation.splines.QuinticHermiteSpline;

public class QuinticHermitePath extends Path {
    /**
     * Constructs a {@link QuinticHermitePath} given an array of waypoint {@link Transform}s.
     *
     * @param waypoints an array of {@link Transform}s that act as waypoints for the path to pass through.
     */
    public QuinticHermitePath(TransformWithVelocityAndCurvature[] waypoints) {
        super(waypoints);
    }

    /**
     * Constructs a {@link QuinticHermitePath} given an array of waypoint {@link TransformWithVelocity}s.
     *
     * @param waypoints an array of {@link TransformWithVelocity}s that act as waypoints for the path to pass through.
     */
    public QuinticHermitePath(TransformWithVelocity[] waypoints) {
        super(waypoints);
    }

    /**
     * Generates the {@link QuinticHermitePath} parametric equations based on the waypoints.
     * <p>
     * It creates a new path for every 2 waypoints, connecting them together.
     */
    @Override
    public void generateParametricEquations() {
        QuinticHermiteSpline[] splines = new QuinticHermiteSpline[getWaypoints().length - 1];
        for (int i = 0; i < getWaypoints().length - 1; i++) {
            splines[i] = new QuinticHermiteSpline(getWaypoints()[i], getWaypoints()[i + 1]);
        }
        setParametrics(splines);
    }

    @Override
    public Path updatePathFromPoints(TransformWithVelocityAndCurvature[] waypoints) {
        return new QuinticHermitePath(waypoints);
    }

}
