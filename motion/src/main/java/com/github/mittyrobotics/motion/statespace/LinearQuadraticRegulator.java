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

public class LinearQuadraticRegulator {
    /**
     * {@link Plant} discrete A matrix.
     */
    private SimpleMatrix A;
    /**
     * {@link Plant} discrete B matrix.
     */
    private SimpleMatrix B;
    /**
     * LQR Controller gain matrix.
     */
    private SimpleMatrix lqrGain;
    /**
     * LQR Controller feedforward gain matrix.
     */
    private SimpleMatrix lqrFFGain;
    /**
     * Controller reference state matrix.
     */
    private SimpleMatrix r;
    /**
     * Control input matrix (also known as controller output, voltage to apply to motor). Feedforward matrix,
     * <code>uff</code> is already applied to this matrix.
     */
    private SimpleMatrix u;
    /**
     * Control input feedforward matrix.
     */
    private SimpleMatrix uff;

    public LinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B, SimpleMatrix qElms, SimpleMatrix rElms, double qWeight) {
        this.A = A;
        this.B = B;

        //Create cost matrices of q and r elements
        SimpleMatrix Q = MatrixUtils.makeCostMatrix(qElms, qWeight);
        SimpleMatrix R = MatrixUtils.makeCostMatrix(rElms, 1);

        //Solve discrete algrbraic riccati equation
        SimpleMatrix S = MatrixUtils.discreteAlgebraicRiccatiEquation(A, B, Q, R);

        //Calculate LQR gain and LQR feedforward gain
        SimpleMatrix temp = B.transpose().mult(S).mult(B).plus(R);
        this.lqrGain = temp.solve(B.transpose().mult(S).mult(A));
        this.lqrFFGain = B.pseudoInverse();

        int states = B.numRows();
        int inputs = B.numCols();

        //Initiate initial reference, control input, and control input feedforward matrices
        this.r = new SimpleMatrix(states, 1);
        this.u = new SimpleMatrix(inputs, 1);
        this.uff = new SimpleMatrix(inputs, 1);

        reset();
    }

    public LinearQuadraticRegulator(Plant plant, SimpleMatrix qElms, SimpleMatrix rElms, SimpleMatrix lqrGain, double qWeight) {
        this(plant, qElms, rElms, qWeight);
        this.lqrGain = lqrGain;
    }

    public LinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B, SimpleMatrix qElms, SimpleMatrix rElms,
                                    SimpleMatrix lqrGain, double qWeight) {
        this(A, B, qElms, rElms, qWeight);
        this.lqrGain = lqrGain;
    }

    public LinearQuadraticRegulator(Plant plant, SimpleMatrix qElms, SimpleMatrix rElms, double qWeight) {
        this(plant.getDiscreteSystem().getA(), plant.getDiscreteSystem().getB(), qElms, rElms, qWeight);
    }

    /**
     * Updates the {@link LinearQuadraticRegulator}.
     *
     * @param currentState the current state matrix of the system.
     */
    public void update(SimpleMatrix currentState) {
        uff = lqrFFGain.mult(r.minus(A.mult(r)));
        SimpleMatrix u = lqrGain.mult(r.minus(currentState));
        this.u = u.plus(uff);
    }

    /**
     * Updates the {@link LinearQuadraticRegulator} and sets the next reference to <code>reference</code>.
     *
     * @param currentState  the current state matrix of the system.
     * @param nextReference the next reference matrix.
     */
    public void update(SimpleMatrix currentState, SimpleMatrix nextReference) {
        update(currentState);
        this.r = nextReference;
    }

    public void setReference(SimpleMatrix r) {
        this.r = r;
    }

    /**
     * Resets the {@link LinearQuadraticRegulator}.
     */
    public void reset() {
        r.fill(0.0);
        u.fill(0.0);
        uff.fill(0.0);
    }

    public SimpleMatrix getLqrGain() {
        return lqrGain;
    }

    public SimpleMatrix getR() {
        return r;
    }

    public SimpleMatrix getU() {
        return u;
    }

    public SimpleMatrix getUff() {
        return uff;
    }

    public SimpleMatrix getLqrFFGain() {
        return lqrFFGain;
    }
}
