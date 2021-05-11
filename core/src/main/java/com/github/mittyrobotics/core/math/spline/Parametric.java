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

package com.github.mittyrobotics.core.math.spline;


import com.github.mittyrobotics.core.math.geometry.Position;
import com.github.mittyrobotics.core.math.geometry.Rotation;
import com.github.mittyrobotics.core.math.geometry.Transform;

public abstract class Parametric {
    /**
     * Returns the {@link Position} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
     *
     * @param t the parameter
     * @return the {@link Position} at the parameter <code>t</code>.
     */
    public abstract Position getPosition(double t);

    /**
     * Returns the {@link Rotation} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
     *
     * @param t the parameter
     * @return the {@link Rotation} at the parameter <code>t</code>.
     */
    public abstract Rotation getRotation(double t);

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
    public double getRawLength(double steps, double startT, double endT) {
        double length = 0;
        for (double t = startT; t < endT; t += (endT - startT) / steps) {
            length += getPosition(t).distance(getPosition(t - (endT - startT) / steps));
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
        return getGaussianQuadratureLength(1);
    }

    /**
     * Computes the estimated length of the parametric using 5-point Gaussian quadrature.
     * <p>
     * https://en.wikipedia.org/wiki/Gaussian_quadrature
     *
     * @param endParam the ending parameter of the parametric.
     * @return the estimated length of the parametric.
     */
    public double getGaussianQuadratureLength(double endParam) {
        return getGaussianQuadratureLength(0, endParam);
    }


    public double getGaussianQuadratureLength(double startParam, double endParam) {
        //11-point Gaussian quadrature coefficients
        double[][] coefficients = {
                {0.0000000000000000, 0.2729250867779006},
                {-0.2695431559523450, 0.2628045445102467},
                {0.2695431559523450, 0.2628045445102467},
                {-0.5190961292068118, 0.2331937645919905},
                {0.5190961292068118, 0.2331937645919905},
                {-0.7301520055740494, 0.1862902109277343},
                {0.7301520055740494, 0.1862902109277343},
                {-0.8870625997680953, 0.1255803694649046},
                {0.8870625997680953, 0.1255803694649046},
                {-0.9782286581460570, 0.0556685671161737},
                {0.9782286581460570, 0.0556685671161737},
        };

        double halfParam = (endParam - startParam) / 2.0;

        double length = 0;
        for (int i = 0; i < coefficients.length; i++) {
            double alpha = startParam + halfParam * (1 + coefficients[i][0]);
            length += getFirstDerivative(alpha).magnitude() * coefficients[i][1];
        }

        return length * halfParam;
    }


    /**
     * Returns the parameter of the parametric at the length along the spline.
     * <p>
     * https://en.wikipedia.org/wiki/Newton%27s_method
     *
     * @param length length along the spline to get the parameter.
     * @return the parameter of the parametric at the length along the spline.
     */
    public double getParameterFromLength(double length) {
        return getParameterFromLength(length, getGaussianQuadratureLength());
    }

    /**
     * Returns the parameter of the parametric at the length along the spline.
     * <p>
     * https://en.wikipedia.org/wiki/Newton%27s_method
     *
     * @param length length along the spline to get the parameter.
     * @return the parameter of the parametric at the length along the spline.
     */
    public double getParameterFromLength(double length, double splineLength) {
        //Initial guess for the t value
        double t = length / splineLength;

        //Newton-Raphson iterations to make more accurate estimation
        for (int i = 0; i < 5; i++) {
            double tangentMagnitude = getFirstDerivative(t).magnitude();
            if (tangentMagnitude > 0.0) {
                t -= (getGaussianQuadratureLength(t) - length) / tangentMagnitude;
                t = Math.min(1, Math.max(t, -1));
            }
        }

        return t;
    }
}
