/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion.statespace;

import org.ejml.simple.SimpleMatrix;

public class LinearQuadraticRegulator {
    public static SimpleMatrix linearQuadraticRegulator(StateSpaceSystem system, SimpleMatrix qElms, SimpleMatrix rElms,
                                                        double qWeight) {
        return linearQuadraticRegulator(system.getA(), system.getB(), qElms, rElms, qWeight);
    }

    public static SimpleMatrix linearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B, SimpleMatrix qElms,
                                                        SimpleMatrix rElms, double qWeight) {
        //Create cost matrices of q and r elements
        SimpleMatrix Q = MatrixUtils.makeCostMatrix(qElms, qWeight);
        SimpleMatrix R = MatrixUtils.makeCostMatrix(rElms, 1);

        //Solve discrete algrbraic riccati equation
        SimpleMatrix S = MatrixUtils.discreteAlgebraicRiccatiEquation(A, B, Q, R);

        //Calculate LQR gain and LQR feedforward gain
        SimpleMatrix temp = B.transpose().mult(S).mult(B).plus(R);
        SimpleMatrix K = temp.solve(B.transpose().mult(S).mult(A));

        return K;
    }
}
