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

package com.github.mittyrobotics.motion.modeling;

import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleMatrix;

import java.util.List;

public class MatrixUtils {
    public static List<Complex_F64> eigenValues(SimpleMatrix input) {
        return input.eig().getEigenvalues();
    }

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
     * https://github.com/jblas-project/jblas/blob/89fde54b386a0f440d7caa1801472ff24820348f/src/main/java/org/jblas/MatrixFunctions.java#L406
     *
     * @param input
     * @return
     */
    public static SimpleMatrix expm(SimpleMatrix input) {
        return SimpleMatrixUtils.expm(input);
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
}
