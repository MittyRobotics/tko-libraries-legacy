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

package com.github.mittyrobotics.datatypes.motion;

public class DrivetrainWheelState {
    private final double left;
    private final double right;

    /**
     * Represents a left and right drivetrain wheel speed. Speed can be represented in any unit, most commonly used
     * as velocity or voltage.
     *
     * @param left  the left wheel speed
     * @param right the right wheel speed
     */
    public DrivetrainWheelState(double left, double right) {
        if (Double.isNaN(left) || Double.isInfinite(left)) {
            this.left = 0;
        } else {
            this.left = left;
        }
        if (Double.isNaN(right) || Double.isInfinite(right)) {
            this.right = 0;
        } else {
            this.right = right;
        }
    }

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public double getAvg() {
        return (right + left) / 2;
    }

    @Override
    public String toString() {
        return String.format("DrivetrainWheelSpeeds(left: %s, right: %s)", left, right);
    }
}
