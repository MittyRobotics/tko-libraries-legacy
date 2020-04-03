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

package com.github.mittyrobotics.motion.controllers;

import org.ejml.simple.SimpleMatrix;

public class ElevatorStateSpaceController extends StateSpaceController {
    public ElevatorStateSpaceController(double startPosition, double startVelocity, double referencePosition,
                                        double referenceVelocity, SimpleMatrix A, SimpleMatrix B, SimpleMatrix C,
                                        SimpleMatrix D, SimpleMatrix K, SimpleMatrix Kff, SimpleMatrix kalmanGain) {
        super(
                new SimpleMatrix(new double[][]{
                        {referencePosition},
                        {referenceVelocity}
                }),
                A, B, C, D, K, Kff,
                new SimpleMatrix(new double[][]{
                        {startPosition},
                        {startVelocity}
                }),
                new SimpleMatrix(new double[][]{
                        {0}
                }),
                kalmanGain);
    }

    public double calculate(double measuredPosition, double referencePosition,
                            double referenceVelocity) {
        return calculate(new SimpleMatrix(new double[][]{
                        {measuredPosition}
                }),
                new SimpleMatrix(new double[][]{
                        {referencePosition},
                        {referenceVelocity}
                })).get(0);
    }

    public double calculate(double measuredPosition) {
        return calculate(measuredPosition, getR().get(0), getR().get(1));
    }

    public double getControllerVoltage() {
        return getU().get(0);
    }

    public double getEstimatedPosition() {
        return getxHat().get(0);
    }

    public double getEstimatedVelocity() {
        return getxHat().get(1);
    }

    public void setReference(double position, double velocity) {
        SimpleMatrix r = getR();
        r.set(0, position);
        r.set(1, velocity);
        setReference(r);
    }

    public boolean isFinished(double positionTolerance, double velocityTolerance) {
        return Math.abs(getError().get(0)) < positionTolerance && Math.abs(getError().get(1)) < velocityTolerance;
    }
}
