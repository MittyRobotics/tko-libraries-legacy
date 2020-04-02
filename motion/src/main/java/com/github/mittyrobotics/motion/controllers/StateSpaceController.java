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

public class StateSpaceController {
    /**
     * Reference matrix
     */
    private SimpleMatrix r;
    /**
     * A gains matrix
     */
    private SimpleMatrix A;
    /**
     * B gains matrix
     */
    private SimpleMatrix B;
    /**
     * C gains matrix
     */
    private SimpleMatrix C;
    /**
     * D gains matrix
     */
    private SimpleMatrix D;
    /**
     * K gains matrix
     */
    private SimpleMatrix K;
    /**
     * K feed forward gains matrix
     */
    private SimpleMatrix Kff;
    /**
     * New predicted state
     */
    private SimpleMatrix xHat;
    private SimpleMatrix x;
    /**
     * Previous state
     */
    private SimpleMatrix y;
    /**
     * Input matrix (input to motors, voltage)
     */
    private SimpleMatrix u;
    /**
     * Kalman filter state estimation gain matrix
     */
    private SimpleMatrix kalmanGain;
    /**
     * Minimum input (-12 volts)
     */
    private double uMin = -12;
    /**
     * Maximum input (12 volts)
     */
    private double uMax = 12;

    public StateSpaceController(SimpleMatrix r, SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix D,
                                SimpleMatrix K, SimpleMatrix Kff, SimpleMatrix xHat, SimpleMatrix u,
                                SimpleMatrix kalmanGain) {
        this.r = r;
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;
        this.K = K;
        this.Kff = Kff;
        this.xHat = xHat;
        this.u = u;
        this.kalmanGain = kalmanGain;
    }

    public SimpleMatrix calculate(SimpleMatrix currentState, SimpleMatrix referenceState) {
        updatePlant();
        correctObserver(currentState);
        updateController(referenceState);
        predictObserver();
        return u;
    }

    public SimpleMatrix calculate(SimpleMatrix currentState) {
        return calculate(currentState, r);
    }

    public void updatePlant() {
        x = A.mult(x).plus(B.mult(u));
        y = C.mult(x).plus(D.mult(u));
    }

    public void correctObserver(SimpleMatrix y) {
        xHat = xHat.plus(kalmanGain.mult(y.minus(C.mult(xHat).minus(D.mult(u)))));
    }

    public void updateController(SimpleMatrix nextR) {
        SimpleMatrix _u = K.mult(nextR.minus(xHat));
        SimpleMatrix uff = Kff.mult(nextR.minus(A.mult(r)));
        r = nextR;
        u = clip(_u.plus(uff), uMin, uMax);
    }

    public void predictObserver() {
        xHat = A.mult(xHat).plus(B.mult(u));
    }

    public SimpleMatrix clip(SimpleMatrix a, double min, double max) {
        for (int row = 0; row < a.numRows(); row++) {
            for (int col = 0; col < a.numCols(); col++) {
                a.set(row, col, Math.max(min, Math.min(max, a.get(row, col))));
            }
        }
        return a;
    }

    /**
     * Sets the reference of the state space controller.
     *
     * @param reference the reference of the state space controller.
     */
    public void setReference(SimpleMatrix reference) {
        this.r = reference;
    }

    /**
     * Sets the output range of the state-space controller.
     *
     * @param minOutput minimum output.
     * @param maxOutput maximum output.
     */
    public void setOutputRange(double minOutput, double maxOutput) {
        this.uMin = minOutput;
        this.uMax = maxOutput;
    }

    /**
     * Resets the state-space controller.
     */
    public void reset() {
        r.zero();
        y.zero();
        y.zero();
        xHat.zero();
    }

    public SimpleMatrix getError() {
        return r.minus(xHat);
    }

    /**
     * Returns the reference matrix.
     *
     * @return the reference matrix.
     */
    public SimpleMatrix getR() {
        return r;
    }

    public SimpleMatrix getA() {
        return A;
    }

    public SimpleMatrix getB() {
        return B;
    }

    public SimpleMatrix getC() {
        return C;
    }

    public SimpleMatrix getD() {
        return D;
    }

    public SimpleMatrix getK() {
        return K;
    }

    public SimpleMatrix getKff() {
        return Kff;
    }

    public SimpleMatrix getxHat() {
        return xHat;
    }

    public SimpleMatrix getX() {
        return x;
    }

    public SimpleMatrix getY() {
        return y;
    }

    public SimpleMatrix getU() {
        return u;
    }

    public SimpleMatrix getKalmanGain() {
        return kalmanGain;
    }

    public double getuMin() {
        return uMin;
    }

    public double getuMax() {
        return uMax;
    }
}