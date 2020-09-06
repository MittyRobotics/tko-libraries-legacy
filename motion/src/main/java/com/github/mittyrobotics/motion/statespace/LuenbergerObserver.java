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

public class LuenbergerObserver {
    /**
     * Calculates the estimation of the state space x dot matrix.
     *
     * @param system discretized state space system.
     * @param xHat previous estimation of x matrix.
     * @param u input matrix.
     * @param L Luenberger observer gain matrix.
     * @param y measurement matrix.
     * @param yHat previous measurement estimation matrix.
     * @return estimation of state matrix, xDotHat.
     */
    public static SimpleMatrix calculateXDotHat(StateSpaceSystem system, SimpleMatrix xHat, SimpleMatrix u, SimpleMatrix L, SimpleMatrix y, SimpleMatrix yHat){
        return system.getA().mult(xHat).plus(system.getB().mult(u)).plus(L.mult(y.minus(yHat)));
    }

    /**
     * Calculates the estimation of the state space y matrix.
     *
     * @param system discretized state space system.
     * @param xHat previous estimation of x matrix.
     * @param u input matrix.
     * @return estimation of measurement matrix, yHat.
     */
    public static SimpleMatrix calculateYHat(StateSpaceSystem system, SimpleMatrix xHat, SimpleMatrix u){
        return system.getC().mult(xHat).plus(system.getD().mult(u));
    }

    /**
     * Predicts next output estimation, xHat.
     *
     * @param system discretized state space system.
     * @param xHat previous estimation of x matrix.
     * @param u input matrix.
     * @return estimation of next output matrix, xHat.
     */
    public static SimpleMatrix predict(StateSpaceSystem system, SimpleMatrix xHat, SimpleMatrix u){
        return system.getA().mult(xHat).plus(system.getB().mult(u));
    }

    /**
     * Corrects the xHat estimation matrix using the measured states from the physical system.
     *
     * @param system discretized state space system.
     * @param xHat previous estimation of x matrix.
     * @param L Luenberger observer gain matrix.
     * @param y input matrix.
     * @param yHat previous measurement estimation.
     * @return corrected xHat matrix.
     */
    public static SimpleMatrix correctX(StateSpaceSystem system, SimpleMatrix xHat, SimpleMatrix L, SimpleMatrix y, SimpleMatrix yHat){
        return xHat.plus(system.getA().pseudoInverse().mult(L).mult(y.minus(yHat)));
    }

    /**
     * Corrects the yHat estimation matrix using the measured states from the physical system.
     *
     * @param system discretized state space system.
     * @param xHat current x matrix estimation.
     * @return corrected yHat matrix.
     */
    public static SimpleMatrix correctY(StateSpaceSystem system, SimpleMatrix xHat){
        return system.getC().mult(xHat);
    }
}
