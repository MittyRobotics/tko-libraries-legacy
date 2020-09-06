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
import org.jblas.util.Random;

public class StateSpace {
    public static ControlInputRange MAX_VOLTAGE = new ControlInputRange(-12, 12);

    /**
     * Applies the closed loop control law to derive input u from state matrix x, reference matrix r, and
     * controller gain matrix K.
     *
     * @param x state matrix
     * @param r reference matrix
     * @param K controller gain matrix
     * @return input matrix u
     */
    public static SimpleMatrix closedLoopControlLaw(SimpleMatrix x, SimpleMatrix r, SimpleMatrix K, ControlInputRange range) {
        SimpleMatrix u = K.mult(r.minus(x));
        return MatrixUtils.clamp(u, range.min, range.max);
    }

    public static SimpleMatrix closedLoopFeedforwardControlLaw(StateSpaceSystem system, SimpleMatrix r, SimpleMatrix Kff, ControlInputRange range){
        SimpleMatrix uff = Kff.mult(r.minus(system.getA().mult(r)));
        return MatrixUtils.clamp(uff, range.min, range.max);
    }

    public static SimpleMatrix calculateFeedforwardGains(StateSpaceSystem system){
        return calculateFeedforwardGains(system.getB());
    }

    public static SimpleMatrix calculateFeedforwardGains(SimpleMatrix B){
        return B.pseudoInverse();
    }

    public static SimpleMatrix applyFeedforward(SimpleMatrix u, ControlInputRange range, SimpleMatrix... uffs){
        for(SimpleMatrix uff : uffs){
            u = u.plus(uff);
        }
        return  MatrixUtils.clamp(u, range.getMin(), range.getMax());
    }

    public static double gaussianDistribution(double mean, double standardDeviation){
        return (Random.nextGaussian()*standardDeviation)+mean;
    }

    public static SimpleMatrix gaussianDistribution(SimpleMatrix input, double standardDeviation){
        SimpleMatrix output = input;
        for(int i = 0; i < input.getNumElements(); i++){
            output.set(i, gaussianDistribution(input.get(0), standardDeviation));
        }
        return output;
    }

    public static class ControlInputRange{
        private double min;
        private double max;

        public ControlInputRange(double min, double max){
            this.min = min;
            this.max = max;
        }

        public double getMin() {
            return min;
        }

        public double getMax() {
            return max;
        }
    }
}
