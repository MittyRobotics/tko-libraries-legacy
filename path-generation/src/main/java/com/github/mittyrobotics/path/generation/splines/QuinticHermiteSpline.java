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

package com.github.mittyrobotics.path.generation.splines;

import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class QuinticHermiteSpline implements Parametric {
    private double x0, x1, y0, y1, vx0, vx1, vy0, vy1, ax0, ax1, ay0, ay1;

    public QuinticHermiteSpline(Transform startWaypoint, Transform endWaypoint) {
        initSpline(startWaypoint, endWaypoint);
    }

    private void initSpline(Transform startWaypoint, Transform endWaypoint) {
        x0 = startWaypoint.getPosition().getX();
        x1 = endWaypoint.getPosition().getX();
        y0 = startWaypoint.getPosition().getY();
        y1 = endWaypoint.getPosition().getY();

        //Get angles in radians
        double heading0 = Math.toRadians(startWaypoint.getRotation().getHeading());
        double heading1 = Math.toRadians(endWaypoint.getRotation().getHeading());

        double d = startWaypoint.getPosition().distance(endWaypoint.getPosition());

        //Create tangent vectors proportional to the distance between points
        vx0 = Math.cos(heading0) * d;
        vy0 = Math.sin(heading0) * d;
        vx1 = Math.cos(heading1) * d;
        vy1 = Math.sin(heading1) * d;

        //Create acceleration vectors
        ax0 = 0;
        ay0 = 0;
        ax1 = 0;
        ay1 = 0;
    }

    @Override
    public Position getPosition(double t) {
        //Quintic hermite spline equations https://rose-hulman.edu/~finn/CCLI/Notes/day09.pdf#page=4
        double h0 = -6 * t * t * t * t * t + 15 * t * t * t * t - 10 * t * t * t + 1;
        double h1 = -3 * t * t * t * t * t + 8 * t * t * t * t - 6 * t * t * t + t;
        double h2 = -(t * t * t * t * t) / 2 + (3 * t * t * t * t) / 2 - (3 * t * t * t) / 2 + (t * t) / 2;
        double h3 = (t * t * t * t * t) / 2 - t * t * t * t + (t * t * t) / 2;
        double h4 = -3 * t * t * t * t * t + 7 * t * t * t * t - 4 * t * t * t;
        double h5 = 6 * t * t * t * t * t - 15 * t * t * t * t + 10 * t * t * t;

        return computeFromCoefficients(h0, h1, h2, h3, h4, h5);
    }

    @Override
    public Transform getTransform(double t) {
        Position position = getPosition(t);
        Position firstDerivative = getFirstDerivative(t);
        Rotation rotation = new Rotation(Math.toDegrees(Math.atan2(firstDerivative.getY(), firstDerivative.getX())));

        return new Transform(position, rotation);
    }

    @Override
    public double getCurvature(double t) {
        Position firstDerivative = getFirstDerivative(t);
        Position secondDerivative = getSecondDerivative(t);

        return (firstDerivative.getX() * secondDerivative.getY() - secondDerivative.getX() * firstDerivative.getY()) /
                Math.sqrt(Math.pow(firstDerivative.getX() * firstDerivative.getX() +
                        firstDerivative.getY() * firstDerivative.getY(), 3));
    }

    @Override
    public Position getFirstDerivative(double t) {
        //First derivative of quintic hermite spline functions
        double h0 = -30 * t * t * t * t + 60 * t * t * t - 30 * t * t;
        double h1 = -15 * t * t * t * t + 32 * t * t * t - 18 * t * t + 1;
        double h2 = -(5 * t * t * t * t) / 2 + 6 * t * t * t - (9 * t * t) / 2 + t;
        double h3 = (5 * t * t * t * t) / 2 - (4 * t * t * t) + (3 * t * t) / 2;
        double h4 = -15 * t * t * t * t + 28 * t * t * t - 12 * t * t;
        double h5 = 30 * t * t * t * t - 60 * t * t * t + 30 * t * t;

        return computeFromCoefficients(h0, h1, h2, h3, h4, h5);
    }

    @Override
    public Position getSecondDerivative(double t) {
        //Second derivative of quintic hermite spline functions
        double h0 = -120 * t * t * t + 180 * t * t - 60 * t;
        double h1 = -60 * t * t * t + 96 * t * t - 36 * t;
        double h2 = -10 * t * t * t + 18 * t * t - 9 * t + 1;
        double h3 = t * (10 * t * t - 12 * t + 3);
        double h4 = -60 * t * t * t + 84 * t * t - 24 * t;
        double h5 = 120 * t * t * t - 180 * t * t + 60 * t;

        return computeFromCoefficients(h0, h1, h2, h3, h4, h5);
    }

    private Position computeFromCoefficients(double h0, double h1, double h2, double h3, double h4, double h5) {
        double x = h0 * x0 + h1 * vx0 + h2 * ax0 + h3 * ax1 + h4 * vx1 + h5 * x1;
        double y = h0 * y0 + h1 * vy0 + h2 * ay0 + h3 * ay1 + h4 * vy1 + h5 * y1;
        return new Position(x, y);
    }
}
