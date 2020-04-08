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
    private SimpleMatrix p;
    private SimpleMatrix contQ;
    private SimpleMatrix contR;
    private SimpleMatrix discR;
    private SimpleMatrix discQ;

    public KalmanFilter() {

    }

    public KalmanFilter(Plant plant, SimpleMatrix stateDeviation, SimpleMatrix measurementDeviation) {
        this.plant = plant;
        this.contQ = makeCovarianceMatrix(stateDeviation);
        this.contR = makeCovarianceMatrix(measurementDeviation);

        this.discR = MatrixUtils.multByDouble(contR, 1/plant.getDeltaTime());

        this.kalmanGain = computeKalmanGain(new StateSpaceSystemGains(plant.getDiscreteSystem().getA(),
                new SimpleMatrix(new double[][]{{0}})
                , plant.getDiscreteSystem().getC(),
                new SimpleMatrix(new double[][]{{0}})), contQ, discR);
    }

    private SimpleMatrix computeKalmanGain(StateSpaceSystemGains discreteSystemGains, SimpleMatrix Q, SimpleMatrix R) {
        this.p = MatrixUtils.discreteAlgebraicRiccatiEquation(discreteSystemGains.getA().transpose(),
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

        StateSpaceSystemGains reDiscretizedSystem = plant.discretizeSystem(plant.getContinuousSystem(), deltaTime);

        SimpleMatrix discA = reDiscretizedSystem.getA();

        this.p = discA.mult(p).mult(discA.transpose()).plus(contQ);
        this.discR = MatrixUtils.multByDouble(contR, 1 / deltaTime);
    }

    public void correct(SimpleMatrix controlInput, SimpleMatrix measurement) {
        SimpleMatrix x = plant.getX();
        SimpleMatrix S =
                plant.getDiscreteSystem().getC().mult(p).mult(plant.getDiscreteSystem().getC().transpose()).plus(contR);

        SimpleMatrix K = S.transpose().solve(plant.getDiscreteSystem().getC().mult(p.transpose())).transpose();

        System.out.println(plant.getDiscreteSystem().getC().mult(x).plus(plant.getDiscreteSystem().getD().mult(controlInput)));

        plant.setX(x.plus(new SimpleMatrix(K.mult(measurement.minus(
                plant.getDiscreteSystem().getC().mult(x)
                        .plus(plant.getDiscreteSystem().getD().mult(controlInput)))))));

        p = SimpleMatrix.identity(plant.getNumStates())
                .minus(K.mult(plant.getDiscreteSystem().getC())).mult(p);
    }

    public SimpleMatrix getXhat(){
        return plant.getX();
    }

    public SimpleMatrix getKalmanGain() {
        return kalmanGain;
    }
}