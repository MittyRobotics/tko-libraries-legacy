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

public class StateSpaceSystem {
    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix C;
    private SimpleMatrix D;
    private double dt;

    public StateSpaceSystem(StateSpaceSystem system) {
        this.A = system.getA();
        this.B = system.getB();
        this.C = system.getC();
        this.D = system.getD();
        this.dt = system.getDt();
    }

    public StateSpaceSystem(double[][] A, double[][] B, double[][] C, double[][] D) {
        this(new SimpleMatrix(A), new SimpleMatrix(B), new SimpleMatrix(C), new SimpleMatrix(D));
    }

    public StateSpaceSystem(double[][] A, double[][] B, double[][] C, double[][] D, double dt) {
        this(new SimpleMatrix(A), new SimpleMatrix(B), new SimpleMatrix(C), new SimpleMatrix(D), dt);
    }

    public StateSpaceSystem(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix D) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;
    }

    public StateSpaceSystem(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix D, double dt) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;
        this.dt = dt;
        discretizeSystem(dt);
    }

    public static StateSpaceSystem sample(StateSpaceSystem continuousSystem, double dt) {
        return new StateSpaceSystem(continuousSystem).sample(dt);
    }

    public StateSpaceSystem sample(double dt) {
        StateSpaceSystem discreteSystem = new StateSpaceSystem(A, B, C, D);
        discreteSystem.discretizeSystem(dt);
        return discreteSystem;
    }

    public void discretizeSystem(double dt) {
//        SimpleMatrix dtA = MatrixUtils.multByDouble(A,dt);
//        SimpleMatrix dtB = MatrixUtils.multByDouble(B,dt);

//        int size = dtB.numRows() + dtB.numCols();
//        SimpleMatrix block = MatrixUtils.hStack(dtA, dtB);
//        block = MatrixUtils.vStack(block, new SimpleMatrix(size-block.numRows(), size-block.numCols()));
//        block = MatrixUtils.expm(block);
//        this.A = MatrixUtils.cut(0, dtA.numRows(), 0, dtA.numCols(), block);
//        this.B = MatrixUtils.cut(0, dtB.numRows(), dtA.numCols(), dtA.numCols()+dtB.numCols(), block);


        SimpleMatrix emUpper = MatrixUtils.hStack(A, B);
        SimpleMatrix lowerA = new SimpleMatrix(new double[B.numCols()][A.numRows()]);
        lowerA.zero();
        SimpleMatrix lowerB = new SimpleMatrix(new double[B.numCols()][B.numCols()]);
        lowerB.zero();
        SimpleMatrix emLower = MatrixUtils.hStack(lowerA, lowerB);


        SimpleMatrix em = MatrixUtils.vStack(emUpper, emLower);

        SimpleMatrix ms = MatrixUtils.expm(MatrixUtils.multByDouble(em, dt));
        System.out.println(ms);
        ms = MatrixUtils.cut(0, A.numRows(), 0, ms.numCols(), ms);
        SimpleMatrix ad = MatrixUtils.cut(0, ms.numRows(), 0, A.numCols(), ms);
        SimpleMatrix bd = MatrixUtils.cut(0, ms.numRows(), A.numCols(), ms.numCols(), ms);

        this.A = ad;
        this.B = bd;
    }

    /**
     * X dot is the change in states matrix
     *
     * @param x
     * @param u
     * @return
     */
    public SimpleMatrix calculateXDot(SimpleMatrix x, SimpleMatrix u) {
        return A.mult(x).plus(B.mult(u));
    }

    /**
     * Y dot is the outputs matrix (i.e. Calculated measurement from sensors)
     *
     * @param x
     * @param u
     * @return
     */
    public SimpleMatrix calculateY(SimpleMatrix x, SimpleMatrix u) {
        return C.mult(x).plus(D.mult(u));
    }

    public SimpleMatrix getA() {
        return A;
    }

    public void setA(SimpleMatrix a) {
        A = a;
    }

    public SimpleMatrix getB() {
        return B;
    }

    public void setB(SimpleMatrix b) {
        B = b;
    }

    public SimpleMatrix getC() {
        return C;
    }

    public void setC(SimpleMatrix c) {
        C = c;
    }

    public SimpleMatrix getD() {
        return D;
    }

    public void setD(SimpleMatrix d) {
        D = d;
    }

    public double getDt() {
        return dt;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }
}
