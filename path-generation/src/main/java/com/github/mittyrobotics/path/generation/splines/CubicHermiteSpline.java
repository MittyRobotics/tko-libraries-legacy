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

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocity;

public class CubicHermiteSpline implements Parametric {
    private double x0, x1, y0, y1, a0, a1, d, mx0, mx1, my0, my1;

    /**
     * Constructs a {@link CubicHermiteSpline} given the start and end {@link Transform}s.
     *
     * @param startWaypoint the {@link Transform} to start the spline.
     * @param endWaypoint   the {@link Transform} to end the spline.
     */
    public CubicHermiteSpline(Transform startWaypoint, Transform endWaypoint) {
        initSpline(new TransformWithVelocity(startWaypoint,d),new TransformWithVelocity(endWaypoint,0));
    }

    /**
     * Constructs a {@link CubicHermiteSpline} given the start and end {@link Transform}s as well as the magnitude of
     * the start and end tangent vectors.
     *
     * @param startWaypoint the {@link Transform} to start the spline.
     * @param endWaypoint   the {@link Transform} to end the spline.
     * @param m1            the magnitude of the start tangent vector.
     * @param m2            the magnitude of the end tangent vector.
     */
    public CubicHermiteSpline(Transform startWaypoint, Transform endWaypoint, double m1, double m2) {
        initSpline(new TransformWithVelocity(startWaypoint,m1),new TransformWithVelocity(endWaypoint,m2));
    }

    /**
     * Constructs a {@link CubicHermiteSpline} given the start and end {@link TransformWithVelocity}s containing a
     * {@link Transform} and velocity vector magnitude
     *
     * @param startWaypoint the {@link TransformWithVelocity} to start the spline containing a {@link Transform} and
     *                      velocity vector magnitude.
     * @param endWaypoint   the {@link TransformWithVelocity} to end the spline containing a {@link Transform} and
     *                      velocity vector magnitude.
     */
    public CubicHermiteSpline(TransformWithVelocity startWaypoint, TransformWithVelocity endWaypoint) {
        initSpline(startWaypoint,endWaypoint);
    }

    private void initSpline(TransformWithVelocity startWaypoint, TransformWithVelocity endWaypoint){
        x0 = startWaypoint.getPosition().getX();
        x1 = endWaypoint.getPosition().getX();
        y0 = startWaypoint.getPosition().getY();
        y1 = endWaypoint.getPosition().getY();

        //Get angles in radians
        a0 = Math.toRadians(startWaypoint.getRotation().getHeading());
        a1 = Math.toRadians(endWaypoint.getRotation().getHeading());

        double startMagnitude = startWaypoint.getVelocity();
        double endMagnitude = endWaypoint.getVelocity();

        double d = startWaypoint.getPosition().distance(endWaypoint.getPosition());

        if(startMagnitude == 0){
            startMagnitude = d;
        }
        if(endMagnitude == 0){
            endMagnitude = d;
        }

        //Create tangent vectors proportional to the distance between points
        mx0 = Math.cos(a0) * startMagnitude;
        my0 = Math.sin(a0) * startMagnitude;
        mx1 = Math.cos(a1) * endMagnitude;
        my1 = Math.sin(a1) * endMagnitude;
    }

    /**
     * Returns the {@link Position} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
     *
     * @param t the parameter
     * @return the {@link Position} at the parameter <code>t</code>.
     */
    @Override
    public Position getPosition(double t) {
        //Cubic hermite spline equations https://en.wikipedia.org/wiki/Cubic_Hermite_spline
        double h0, h1, h2, h3;
        h0 = 2 * t * t * t - 3 * t * t + 1;
        h1 = t * t * t - 2 * t * t + t;
        h2 = -2 * t * t * t + 3 * t * t;
        h3 = t * t * t - t * t;

        //Get x and y values from cubic hermite spline equations
        double x = h0 * x0 + h1 * mx0 + h2 * x1 + h3 * mx1;
        double y = h0 * y0 + h1 * my0 + h2 * y1 + h3 * my1;
        return new Position(x, y);
    }

    /**
     * Returns the {@link Transform} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
     * <p>
     * The {@link Transform} contains the {@link Position} and {@link Rotation}, with the {@link Rotation} being the
     * tangent angle at the {@link Position}.
     *
     * @param t the parameter
     * @return the {@link Transform} at the parameter <code>t</code>.
     */
    @Override
    public Transform getTransform(double t) {
        Position position = getPosition(t);

        //To get tangent vector equations (first derivative of cubic hermite spline functions)
        double h0, h1, h2, h3;
        h0 = 6 * t * t - 6 * t;
        h1 = 3 * t * t - 4 * t + 1;
        h2 = -6 * t * t + 6 * t;
        h3 = 3 * t * t - 2 * t;

        //Get x and y values for tangent vector from the equations
        double x = h0 * x0 + h1 * mx0 + h2 * x1 + h3 * mx1;
        double y = h0 * y0 + h1 * my0 + h2 * y1 + h3 * my1;

        Rotation rotation = new Rotation(Math.toDegrees(Math.atan2(y, x)));

        return new Transform(position, rotation);
    }

    /**
     * Returns the curvature at point <code>t</code> on the {@link Parametric}.
     *
     * @param t the parameter
     * @return the curvature at the parameter <code>t</code>.
     */
    @Override
    public double getCurvature(double t) {
        if(t <= 0.01){
            Circle circle = new Circle(getPosition(t + 0.01), getPosition(t), getPosition(t + 0.02));
            return 1 / circle.getRadius();
        }
        else if(t >= 9.99){

        }
        Circle circle = new Circle(getPosition(t - 0.01), getPosition(t), getPosition(t + 0.01));
        return 1 / circle.getRadius();
    }
}
