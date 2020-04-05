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
import org.jblas.DoubleMatrix;
import org.jblas.MatrixFunctions;

public class MatrixUtils {
    public static SimpleMatrix expm(SimpleMatrix input){
        DoubleMatrix mat;
        double[][] matDoubles = new double[input.numRows()][input.numCols()];
        for(int c = 0; c < input.numCols(); c++){
            for(int r = 0; r < input.numRows(); r++){
                matDoubles[r][c] = input.get(r,c);
            }
        }
        mat = new DoubleMatrix(matDoubles);
        mat = MatrixFunctions.expm(mat);
        SimpleMatrix output = new SimpleMatrix(new double[mat.rows][mat.columns]);
        for(int c = 0; c < mat.columns; c++){
            for(int r = 0; r < mat.rows; r++){
                output.set(r,c,mat.get(r,c));
            }
        }
        return output;
    }

    public static SimpleMatrix multByDouble(SimpleMatrix matrix, double scalar){
        SimpleMatrix output = new SimpleMatrix(new double[matrix.numRows()][matrix.numCols()]);
        for(int i = 0; i < output.getNumElements(); i++){
            output.set(i, matrix.get(i) * scalar);
        }
        return output;
    }

    public static SimpleMatrix cut(int startRow, int endRow, int startCol, int endCol, SimpleMatrix matrix){
        SimpleMatrix output = new SimpleMatrix(new double[endRow-startRow][endCol-startCol]);
        for(int c = 0; c < output.numCols(); c++){
            for(int r = 0; r < output.numRows(); r++){
                output.set(r,c,matrix.get(r+startRow, c+startCol));
            }
        }
        return output;
    }

    public static SimpleMatrix hStack(SimpleMatrix a, SimpleMatrix b){
        SimpleMatrix output = new SimpleMatrix(new double[a.numRows()][a.numCols()+b.numCols()]);
        for(int c = 0; c < a.numCols(); c++){
            for(int r = 0; r < a.numRows(); r++){
                output.set(r,c,a.get(r,c));
            }
        }
        for(int c = 0; c < b.numCols(); c++){
            for(int r = 0; r < b.numRows(); r++){
                output.set(r,c+a.numCols(),b.get(r,c));
            }
        }

        return output;
    }

    public static SimpleMatrix vStack(SimpleMatrix a, SimpleMatrix b){
        SimpleMatrix output = new SimpleMatrix(new double[a.numRows()+b.numRows()][a.numCols()]);
        for(int c = 0; c < a.numCols(); c++){
            for(int r = 0; r < a.numRows(); r++){
                output.set(r,c,a.get(r,c));
            }
        }
        for(int c = 0; c < b.numCols(); c++){
            for(int r = 0; r < b.numRows(); r++){
                output.set(r+a.numRows(),c,b.get(r,c));
            }
        }

        return output;
    }
}
