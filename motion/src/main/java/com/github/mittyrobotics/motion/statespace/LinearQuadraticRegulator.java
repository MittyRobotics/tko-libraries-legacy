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
    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix k;
    private SimpleMatrix r;
    private SimpleMatrix u;
    private SimpleMatrix uff;

    public LinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B, SimpleMatrix qElms, SimpleMatrix rElms) {
        this.A = A;
        this.B = B;

        SimpleMatrix Q = MatrixUtils.makeCostMatrix(qElms);
        SimpleMatrix R = MatrixUtils.makeCostMatrix(rElms);

        SimpleMatrix S = MatrixUtils.discreteAlgebraicRiccatiEquation(A, B, Q, R);

        SimpleMatrix temp = B.transpose().mult(S).mult(B).plus(R);
        this.k = temp.solve(B.transpose().mult(S).mult(A));

        int states = B.numRows();
        int inputs = B.numCols();

        this.r = new SimpleMatrix(states, 1);
        this.u = new SimpleMatrix(inputs, 1);
        this.uff = new SimpleMatrix(inputs, 1);

        reset();
    }

    public LinearQuadraticRegulator(Plant plant, SimpleMatrix qElms, SimpleMatrix rElms, SimpleMatrix k) {
        this(plant, qElms, rElms);
        this.k = k;
    }

    public LinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B, SimpleMatrix qElms, SimpleMatrix rElms,
                                    SimpleMatrix k) {
        this(A, B, qElms, rElms);
        this.k = k;
    }

    public LinearQuadraticRegulator(Plant plant, SimpleMatrix qElms, SimpleMatrix rElms) {
        this(plant.getDiscreteSystem().getA(), plant.getDiscreteSystem().getB(), qElms, rElms);
    }

    /**
     * Updates the {@link LinearQuadraticRegulator}.
     *
     * @param currentState the current state matrix of the system.
     */
    public void update(SimpleMatrix currentState) {
        uff = new SimpleMatrix(B.solve(r.minus(A.mult(r))));
        u = k.mult(r.minus(currentState)).plus(uff);
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

    /**
     * Resets the {@link LinearQuadraticRegulator}.
     */
    public void reset() {
        r.fill(0.0);
        u.fill(0.0);
        uff.fill(0.0);
    }

    public SimpleMatrix getK() {
        return k;
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
}
