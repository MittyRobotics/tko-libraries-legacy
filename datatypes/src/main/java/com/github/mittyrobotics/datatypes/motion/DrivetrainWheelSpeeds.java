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

public class DrivetrainWheelSpeeds {
    private final double leftSpeed;
    private final double rightSpeed;

    /**
     * Represents a left and right drivetrain wheel speed. Speed can be represented in any unit, most commonly used
     * as velocity or voltage.
     *
     * @param leftSpeed  the left wheel speed
     * @param rightSpeed the right wheel speed
     */
    public DrivetrainWheelSpeeds(double leftSpeed, double rightSpeed) {
        if (Double.isNaN(leftSpeed) || Double.isInfinite(leftSpeed)) {
            this.leftSpeed = 0;
        } else {
            this.leftSpeed = leftSpeed;
        }
        if (Double.isNaN(rightSpeed) || Double.isInfinite(rightSpeed)) {
            this.rightSpeed = 0;
        } else {
            this.rightSpeed = rightSpeed;
        }
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public double getRightSpeed() {
        return rightSpeed;
    }

    public double getAvgSpeed() {
        return (rightSpeed + leftSpeed) / 2;
    }

    @Override
    public String toString() {
        return String.format("DrivetrainWheelSpeeds(left: %s, right: %s)", leftSpeed, rightSpeed);
    }
}
