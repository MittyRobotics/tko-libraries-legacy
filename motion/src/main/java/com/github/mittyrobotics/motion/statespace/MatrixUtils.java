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

import com.github.mittyrobotics.jni.DrakeJNI;
import org.ejml.simple.SimpleMatrix;

public class MatrixUtils {
    public static double normmax(SimpleMatrix input) {
        double max = 0.0;
        for (int i = 0; i < input.getNumElements(); i++) {
            double a = Math.abs(input.get(i));
            if (a > max) {
                max = a;
            }
        }
        return max;
    }

    /**
     * Port of jblas "expm" Matrix Function to use EJML Simple Matrix
     * @param input
     * @return
     */
    public static SimpleMatrix expm(SimpleMatrix input) {
        final double c0 = 1.0;
        final double c1 = 0.5;
        final double c2 = 0.12;
        final double c3 = 0.01833333333333333;
        final double c4 = 0.0019927536231884053;
        final double c5 = 1.630434782608695E-4;
        final double c6 = 1.0351966873706E-5;
        final double c7 = 5.175983436853E-7;
        final double c8 = 2.0431513566525E-8;
        final double c9 = 6.306022705717593E-10;
        final double c10 = 1.4837700484041396E-11;
        final double c11 = 2.5291534915979653E-13;
        final double c12 = 2.8101705462199615E-15;
        final double c13 = 1.5440497506703084E-17;

        int j = Math.max(0, 1 + (int) Math.floor(Math.log(normmax(input)) / Math.log(2)));
        SimpleMatrix As = input.divide((double) Math.pow(2, j)); // scaled version of A
        int n = input.numRows();

        // calculate D and N using special Horner techniques
        SimpleMatrix As_2 = As.mult(As);
        SimpleMatrix As_4 = As_2.mult(As_2);
        SimpleMatrix As_6 = As_4.mult(As_2);
        // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
        SimpleMatrix U = multByDouble(SimpleMatrix.identity(n),c0).plus(multByDouble(As_2,c2)).plus(multByDouble(As_4
                ,c4)).plus(
                multByDouble(SimpleMatrix.identity(n),c6).plus(multByDouble(As_2,c8)).plus(multByDouble(As_4,c10)).plus(multByDouble(As_6,c12)).mult(As_6));
        // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
        SimpleMatrix V =
                multByDouble(SimpleMatrix.identity(n),c1).plus(multByDouble(As_2,c3)).plus(multByDouble(As_4,c5)).plus(
                multByDouble(SimpleMatrix.identity(n),c7).plus(multByDouble(As_2,c9)).plus(multByDouble(As_4,c11)).plus(multByDouble(As_6,c13)).mult(As_6));

        SimpleMatrix AV = As.mult(V);
        SimpleMatrix N = U.plus(AV);
        SimpleMatrix D = U.minus(AV);

        // solve DF = N for F
        SimpleMatrix F = D.solve(N);

        // now square j times
        for (int k = 0; k < j; k++) {
            F.mult(F);
        }

        return F;
    }

    public static SimpleMatrix multByDouble(SimpleMatrix matrix, double scalar) {
        SimpleMatrix output = new SimpleMatrix(new double[matrix.numRows()][matrix.numCols()]);
        for (int i = 0; i < output.getNumElements(); i++) {
            output.set(i, matrix.get(i) * scalar);
        }
        return output;
    }

    public static SimpleMatrix divideByDouble(SimpleMatrix matrix, double scalar) {
        SimpleMatrix output = new SimpleMatrix(new double[matrix.numRows()][matrix.numCols()]);
        for (int i = 0; i < output.getNumElements(); i++) {
            output.set(i, matrix.get(i) / scalar);
        }
        return output;
    }

    public static SimpleMatrix divideDoubleByMatrix(double scalar, SimpleMatrix matrix) {
        SimpleMatrix output = new SimpleMatrix(new double[matrix.numRows()][matrix.numCols()]);
        for (int i = 0; i < output.getNumElements(); i++) {
            output.set(i, scalar / matrix.get(i));
        }
        return output;
    }

    public static SimpleMatrix cut(int startRow, int endRow, int startCol, int endCol, SimpleMatrix matrix) {
        SimpleMatrix output = new SimpleMatrix(new double[endRow - startRow][endCol - startCol]);
        for (int c = 0; c < output.numCols(); c++) {
            for (int r = 0; r < output.numRows(); r++) {
                output.set(r, c, matrix.get(r + startRow, c + startCol));
            }
        }
        return output;
    }

    public static SimpleMatrix hStack(SimpleMatrix a, SimpleMatrix b) {
        SimpleMatrix output = new SimpleMatrix(new double[a.numRows()][a.numCols() + b.numCols()]);
        for (int c = 0; c < a.numCols(); c++) {
            for (int r = 0; r < a.numRows(); r++) {
                output.set(r, c, a.get(r, c));
            }
        }
        for (int c = 0; c < b.numCols(); c++) {
            for (int r = 0; r < b.numRows(); r++) {
                output.set(r, c + a.numCols(), b.get(r, c));
            }
        }

        return output;
    }

    public static SimpleMatrix vStack(SimpleMatrix a, SimpleMatrix b) {
        SimpleMatrix output = new SimpleMatrix(new double[a.numRows() + b.numRows()][a.numCols()]);
        for (int c = 0; c < a.numCols(); c++) {
            for (int r = 0; r < a.numRows(); r++) {
                output.set(r, c, a.get(r, c));
            }
        }
        for (int c = 0; c < b.numCols(); c++) {
            for (int r = 0; r < b.numRows(); r++) {
                output.set(r + a.numRows(), c, b.get(r, c));
            }
        }

        return output;
    }

    public static SimpleMatrix makeCostMatrix(SimpleMatrix cost, double p) {
        return SimpleMatrix.diag(divideDoubleByMatrix(p, cost.elementMult(cost)).getDDRM().getData());
    }

    public static SimpleMatrix clamp(SimpleMatrix matrix, double min, double max) {
        SimpleMatrix output = new SimpleMatrix(matrix.numRows(), matrix.numCols());
        for (int i = 0; i < output.getNumElements(); i++) {
            output.set(i, Math.max(min, Math.min(max, matrix.get(i))));
        }
        return output;
    }

    public static SimpleMatrix discreteAlgebraicRiccatiEquation(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q,
                                                                SimpleMatrix R) {

        int states = A.numCols();
        int inputs = B.numCols();

        double[] resultsArray = DrakeJNI
                .discreteAlgebraicRiccatiEquationJNI(A.getDDRM().getData(), B.getDDRM().getData(), Q.getDDRM().getData(), R.getDDRM().getData(), states, inputs);

        return new SimpleMatrix(states, states, true, resultsArray);
    }
}
