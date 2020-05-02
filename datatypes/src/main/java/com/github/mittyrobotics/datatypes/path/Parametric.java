/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.datatypes.path;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public abstract class Parametric {
    /**
     * Returns the {@link Position} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
     *
     * @param t the parameter
     * @return the {@link Position} at the parameter <code>t</code>.
     */
    public abstract Position getPosition(double t);

    /**
     * Returns the {@link Transform} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
     * <p>
     * The {@link Transform} contains the {@link Position} and {@link Rotation}, with the {@link Rotation} being the
     * tangent angle at the {@link Position}.
     *
     * @param t the parameter
     * @return the {@link Transform} at the parameter <code>t</code>.
     */
    public abstract Transform getTransform(double t);

    /**
     * Returns the curvature at point <code>t</code> on the {@link Parametric}.
     *
     * @param t the parameter
     * @return the curvature at the parameter <code>t</code>.
     */
    public abstract double getCurvature(double t);

    /**
     * Returns the first derivative of the {@link Parametric} in the form of a {@link Position} containing the x and
     * y value of the first derivative at the parameter <code>t</code>.
     *
     * @param t the parameter
     * @return the first derivative {@link Position} at the parameter <code>t</code>.
     */
    public abstract Position getFirstDerivative(double t);

    /**
     * Returns the second derivative of the {@link Parametric} in the form of a {@link Position} containing the x and
     * y value of the second derivative at the parameter <code>t</code>.
     *
     * @param t the parameter
     * @return the second derivative {@link Position} at the parameter <code>t</code>.
     */
    public abstract Position getSecondDerivative(double t);

    /**
     * Computes the estimated length of the parametric by counting the length of each segment for every step. This is
     * slower but more accurate than the Gaussian quatrature method.
     *
     * @param steps the amount of segments it counts. Higher values are more accurate.
     * @return the estimated length of the parametric.
     */
    public double getRawLength(double steps) {
        double length = 0;
        for (double t = 0; t < 1; t += 1 / steps) {
            length += getPosition(t).distance(getPosition(t - 1 / steps));
        }
        return length;
    }

    /**
     * Computes the estimated length of the parametric using 5-point Gaussian quadrature.
     * <p>
     * https://en.wikipedia.org/wiki/Gaussian_quadrature
     *
     * @return the estimated length of the parametric.
     */
    public double getGaussianQuadratureLength() {
        //5-point Gaussian quadrature coefficients
        double[][] coefficients = {
                {0.0, 0.5688889},
                {-0.5384693, 0.47862867},
                {0.5384693, 0.47862867},
                {-0.90617985, 0.23692688},
                {0.90617985, 0.23692688}
        };

        double length = 0;
        for (int i = 0; i < coefficients.length; i++) {
            double t = (1.0 + coefficients[i][0]) / 2;
            length += getFirstDerivative(t).distance(new Position(0, 0)) * coefficients[i][1];
        }
        return length / 2;
    }
}
