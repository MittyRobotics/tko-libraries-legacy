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

public class ContinuousManager {
    private double minInput, maxInput;
    private double inputRange;

    public ContinuousManager(double minInput, double maxInput) {
        setInputRange(minInput, maxInput);
    }

    public ContinuousManager() {
        this(0, 0);
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        inputRange = maxInput - minInput;
    }

    public double getMinInput() {
        return minInput;
    }

    public double getMaxInput() {
        return maxInput;
    }

    public boolean isEnabled() {
        return inputRange > 0;
    }

    public double getInputRange() {
        return inputRange;
    }

    public double getContinousError(double setpoint, double measurement) {
        if (isEnabled() && measurement >= minInput && measurement <= maxInput) {
            double error = (setpoint - measurement) % inputRange;
            if (Math.abs(error) > inputRange / 2) {
                if (error > 0) {
                    return error - inputRange;
                } else {
                    return error + inputRange;
                }
            }
        }
        return setpoint - measurement;
    }

    public double mapValue(double value) {
        if (isEnabled()) {
            value %= inputRange;
            if (value < 0) {
                value += inputRange;
            }
            value += minInput;
        }
        return value;
    }
}