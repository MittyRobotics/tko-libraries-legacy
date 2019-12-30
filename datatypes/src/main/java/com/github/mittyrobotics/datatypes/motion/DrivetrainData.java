/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

public class DrivetrainData {
    private double linearVelocity;
    private double angularVelocity;
    private double leftVelocity;
    private double rightVelocity;

    public DrivetrainData(double linearVelocity, double angularVelocity) {
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;

        DrivetrainVelocities drivetrainVelocities =
                DifferentialDriveKinematics.getInstance().calculateFromAngularVelocity(linearVelocity, angularVelocity);

        this.leftVelocity = drivetrainVelocities.getLeftVelocity();
        this.rightVelocity = drivetrainVelocities.getRightVelocity();
    }

    public DrivetrainData(DrivetrainVelocities drivetrainVelocities) {
        this.linearVelocity = drivetrainVelocities.getAvgVelocity();
        this.angularVelocity =
                DifferentialDriveKinematics.getInstance().getAngularVelocityFromWheelSpeeds(drivetrainVelocities);
        this.leftVelocity = drivetrainVelocities.getLeftVelocity();
        this.rightVelocity = drivetrainVelocities.getRightVelocity();
    }

    public double getLinearVelocity() {
        return linearVelocity;
    }

    public void setLinearVelocity(double linearVelocity) {
        this.linearVelocity = linearVelocity;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public void setLeftVelocity(double leftVelocity) {
        this.leftVelocity = leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public void setRightVelocity(double rightVelocity) {
        this.rightVelocity = rightVelocity;
    }
}
