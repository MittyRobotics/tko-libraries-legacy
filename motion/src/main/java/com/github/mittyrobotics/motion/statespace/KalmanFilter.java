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

package com.github.mittyrobotics.motion.statespace;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    /**
     * Plant that the {@link KalmanFilter} is observing.
     */
    private Plant plant;
    /**
     * Kalman filter gain matrix
     */
    private SimpleMatrix kalmanGain;

    public KalmanFilter(Plant plant, SimpleMatrix stateDeviation, SimpleMatrix measurementDeviation) {
        this.plant = plant;
        SimpleMatrix Q = makeCovarianceMatrix(stateDeviation);
        SimpleMatrix R = makeCovarianceMatrix(measurementDeviation);

        this.kalmanGain = computeKalmanGain(new StateSpaceSystemGains(plant.getDiscreteSystem().getA(),
                new SimpleMatrix(new double[][]{{0}})
                , plant.getDiscreteSystem().getC(),
                new SimpleMatrix(new double[][]{{0}})), Q, R);
    }

    private SimpleMatrix computeKalmanGain(StateSpaceSystemGains discreteSystemGains, SimpleMatrix Q, SimpleMatrix R) {
        SimpleMatrix p = MatrixUtils.discreteAlgebraicRiccatiEquation(discreteSystemGains.getA().transpose(),
                discreteSystemGains.getC().transpose(), Q, R);
        SimpleMatrix s =
                discreteSystemGains.getC().mult(p).mult(discreteSystemGains.getC().transpose()).plus(R);
        SimpleMatrix k = p.mult(discreteSystemGains.getC().transpose()).mult(s.pseudoInverse());
        return k;
    }

    private SimpleMatrix makeCovarianceMatrix(SimpleMatrix standardDeviations) {
        return SimpleMatrix.diag(standardDeviations.elementPower(2).getDDRM().getData());
    }

    public void predict(SimpleMatrix controlInput, double deltaTime) {
        plant.setX(plant.calculateX(plant.getX(), controlInput, deltaTime));
    }

    public void correct(SimpleMatrix controlInput, SimpleMatrix measurement) {
        plant.setX(plant.getX().plus(kalmanGain
                .mult(measurement.minus(plant.getDiscreteSystem().getC().mult(plant.getX()))
                        .minus(plant.getDiscreteSystem().getD().mult(controlInput)))));
    }

    public SimpleMatrix getXhat() {
        return plant.getX();
    }

    public SimpleMatrix getKalmanGain() {
        return kalmanGain;
    }
}