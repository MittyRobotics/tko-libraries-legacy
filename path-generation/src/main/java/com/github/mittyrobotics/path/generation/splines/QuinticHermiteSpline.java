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
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;

/**
 * Quintic Hermite Spline class
 * <p>
 * Reference: https://rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
 * <p>
 * Desmos graph of spline: https://www.desmos.com/calculator/7rxnlvbt2j
 */
public class QuinticHermiteSpline extends Parametric {
    private double x0, x1, y0, y1, vx0, vx1, vy0, vy1, ax0, ax1, ay0, ay1;

    public QuinticHermiteSpline(Transform startWaypoint, Transform endWaypoint) {
        initSpline(startWaypoint, endWaypoint, 0, 0, 0, 0);
    }

    public QuinticHermiteSpline(TransformWithVelocityAndCurvature startWaypoint,
                                TransformWithVelocityAndCurvature endWaypoint) {
        double d = startWaypoint.getPosition().distance(endWaypoint.getPosition());
        double a0 = (startWaypoint.getCurvature()) * (d * d);
        double a1 = (endWaypoint.getCurvature()) * (d * d);
        initSpline(startWaypoint, endWaypoint, Math.sin(startWaypoint.getRotation().getRadians()) * a0,
                Math.cos(startWaypoint.getRotation().getRadians()) * a0,
                Math.sin(endWaypoint.getRotation().getRadians()) * a1,
                Math.cos(endWaypoint.getRotation().getRadians()) * a1);
    }

    public QuinticHermiteSpline(Transform startWaypoint, Transform endWaypoint, double ax0, double ay0, double ax1,
                                double ay1) {
        initSpline(startWaypoint, endWaypoint, ax0, ay0, ax1, ay1);
    }


    private void initSpline(Transform startWaypoint, Transform endWaypoint, double ax0, double ay0, double ax1,
                            double ay1) {
        this.x0 = startWaypoint.getPosition().getX();
        this.x1 = endWaypoint.getPosition().getX();
        this.y0 = startWaypoint.getPosition().getY();
        this.y1 = endWaypoint.getPosition().getY();

        //Get angles in radians
        double heading0 = Math.toRadians(startWaypoint.getRotation().getHeading());
        double heading1 = Math.toRadians(endWaypoint.getRotation().getHeading());

        double d = startWaypoint.getPosition().distance(endWaypoint.getPosition());

        //Create tangent vectors proportional to the distance between points
        this.vx0 = Math.cos(heading0) * d;
        this.vy0 = Math.sin(heading0) * d;
        this.vx1 = Math.cos(heading1) * d;
        this.vy1 = Math.sin(heading1) * d;

        //Create acceleration vectors
        this.ax0 = ax0;
        this.ay0 = ay0;
        this.ax1 = ax1;
        this.ay1 = ay1;
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

    /**
     * Computes the {@link Position} from the 6 base coefficients.
     *
     * @param h0 base coefficient 1
     * @param h1 base coefficient 2
     * @param h2 base coefficient 3
     * @param h3 base coefficient 4
     * @param h4 base coefficient 5
     * @param h5 base coefficient 6
     * @return
     */
    private Position computeFromCoefficients(double h0, double h1, double h2, double h3, double h4, double h5) {
        double x = h0 * x0 + h1 * vx0 + h2 * ax0 + h3 * ax1 + h4 * vx1 + h5 * x1;
        double y = h0 * y0 + h1 * vy0 + h2 * ay0 + h3 * ay1 + h4 * vy1 + h5 * y1;
        return new Position(x, y);
    }
}