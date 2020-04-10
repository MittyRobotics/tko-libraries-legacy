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
    private Plant plant;
    private SimpleMatrix kalmanGain;
    private SimpleMatrix contQ;
    private SimpleMatrix contR;

    public KalmanFilter(Plant plant, SimpleMatrix stateDeviation, SimpleMatrix measurementDeviation) {
        this.plant = plant;
        this.contQ = makeCovarianceMatrix(stateDeviation);
        this.contR = makeCovarianceMatrix(measurementDeviation);

        this.kalmanGain = computeKalmanGain(new StateSpaceSystemGains(plant.getDiscreteSystem().getA(),
                new SimpleMatrix(new double[][]{{0}})
                , plant.getDiscreteSystem().getC(),
                new SimpleMatrix(new double[][]{{0}})), contQ, contR);
    }

    private SimpleMatrix computeKalmanGain(StateSpaceSystemGains discreteSystemGains, SimpleMatrix Q, SimpleMatrix R) {
        SimpleMatrix p = MatrixUtils.discreteAlgebraicRiccatiEquation(discreteSystemGains.getA().transpose(),
                discreteSystemGains.getC().transpose(), Q, R);

        SimpleMatrix s =
                discreteSystemGains.getC().mult(p).mult(discreteSystemGains.getC().transpose()).plus(R);
        SimpleMatrix k = p.mult(discreteSystemGains.getC().transpose()).mult(s.pseudoInverse());
        return k;
    }

    private SimpleMatrix discretizeQ(SimpleMatrix contA, SimpleMatrix discA, SimpleMatrix contQ, double deltaTime) {
        SimpleMatrix Q = (contQ.plus(contQ.transpose())).divide(2.0);

        SimpleMatrix lastTerm = Q.copy();
        double lastCoeff = deltaTime;

        // A^T^n
        SimpleMatrix Atn = contA.transpose();
        SimpleMatrix phi12 = MatrixUtils.multByDouble(lastTerm, lastCoeff);

        // i = 6 i.e. 6th order should be enough precision
        for (int i = 2; i < 6; ++i) {
            lastTerm = MatrixUtils.multByDouble(contA, -1).mult(lastTerm).plus(Q.mult(Atn));
            lastCoeff *= deltaTime / ((double) i);

            phi12 = phi12.plus(MatrixUtils.multByDouble(lastTerm, lastCoeff));

            Atn = Atn.mult(contA.transpose());
        }

        Q = discA.mult(phi12);

        // Make Q symmetric if it isn't already
        var discQ = Q.plus(Q.transpose()).divide(2.0);

        return discQ;
    }

    private SimpleMatrix makeCovarianceMatrix(SimpleMatrix standardDeviations) {
        return SimpleMatrix.diag(standardDeviations.elementPower(2).getDDRM().getData());
    }

    public void predict(SimpleMatrix controlInput, double deltaTime) {
        plant.setX(plant.calculateX(plant.getX(), controlInput, deltaTime));
    }

    public void correct(SimpleMatrix controlInput, SimpleMatrix measurement) {
        SimpleMatrix x = plant.getX();
        plant.setX(x.plus(kalmanGain.mult(measurement.minus(plant.getDiscreteSystem().getC().mult(x)).minus(plant.getDiscreteSystem().getD().mult(controlInput)))));
    }

    public SimpleMatrix getXhat() {
        return plant.getX();
    }

    public SimpleMatrix getKalmanGain() {
        return kalmanGain;
    }
}