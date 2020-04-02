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

public class DrivetrainVelocities {
    private double linear;
    private double angular;
    private double left;
    private double right;
    private double curvature;
    private double trackWidth;

    public DrivetrainVelocities(double linear, double angular, double left, double right, double curvature,
                                double trackWidth) {
        this.linear = linear;
        this.angular = angular;
        this.left = left;
        this.right = right;
        this.curvature = curvature;
        this.trackWidth = trackWidth;
    }

    public static DrivetrainVelocities empty() {
        return new DrivetrainVelocities(0, 0, 0, 0, 0, 0);
    }

    public static DrivetrainVelocities calculateFromLinearMovementAndRadius(double linear, double radius,
                                                                            double trackWidth) {
        DrivetrainWheelSpeeds drivetrainWheelVelocities =
                DifferentialDriveKinematics.calculateFromRadius(linear, radius, trackWidth);
        return calculateFromWheelMovement(drivetrainWheelVelocities, trackWidth);
    }

    public static DrivetrainVelocities calculateFromWheelMovement(double left, double right, double trackWidth) {
        return calculateFromWheelMovement(new DrivetrainWheelSpeeds(left, right), trackWidth);
    }

    public static DrivetrainVelocities calculateFromWheelMovement(
            DrivetrainWheelSpeeds drivetrainWheelVelocities, double trackWidth) {
        //Get linear and angular velocity from drivetrain velocities
        double linear = drivetrainWheelVelocities.getAvgSpeed();
        double angular =
                DifferentialDriveKinematics.getAngularVelocityFromWheelSpeeds(drivetrainWheelVelocities, trackWidth);

        //Get driving curvature from linear velocity and angular velocity
        double curvature = 1 / (linear / angular);

        if (Double.isNaN(curvature)) {
            curvature = 0;
        }

        return new DrivetrainVelocities(linear, angular, drivetrainWheelVelocities.getLeftSpeed(),
                drivetrainWheelVelocities.getRightSpeed(), curvature, trackWidth);
    }

    public static DrivetrainVelocities calculateFromLinearAndAngularVelocity(double linear, double angular,
                                                                             double trackWidth) {
        //Calculate drivetrain velocities from linear and angular velocities
        DrivetrainWheelSpeeds drivetrainWheelVelocities =
                DifferentialDriveKinematics.calculateFromAngularMovement(linear, angular, trackWidth);

        //Get left and right velocity from drivetrain velocities
        double left = drivetrainWheelVelocities.getLeftSpeed();
        double right = drivetrainWheelVelocities.getRightSpeed();

        //Get driving curvature from linear velocity and angular velocity
        double curvature = 1 / (linear / angular);

        return new DrivetrainVelocities(linear, angular, left, right, curvature,
                trackWidth);
    }

    public DrivetrainVelocities reverse() {
        return new DrivetrainVelocities(-linear, angular, -left, -right,
                curvature, trackWidth);
    }

    private void setValues(DrivetrainVelocities data) {
        this.linear = data.getLinear();
        this.angular = data.getAngular();
        this.left = data.getLeft();
        this.right = data.getRight();
        this.curvature = data.getCurvature();
        this.trackWidth = data.getTrackWidth();
    }

    public double getLinear() {
        return linear;
    }

    public void setLinear(double linear) {
        this.linear = linear;
        setValues(calculateFromLinearAndAngularVelocity(linear, angular, trackWidth));
    }

    public double getAngular() {
        return angular;
    }

    public void setAngular(double angular) {
        this.angular = angular;
        setValues(calculateFromLinearAndAngularVelocity(linear, angular, trackWidth));
    }

    public double getLeft() {
        return left;
    }

    public void setLeft(double left) {
        this.left = left;
        setValues(calculateFromWheelMovement(left, right, trackWidth));
    }

    public double getRight() {
        return right;
    }

    public void setRight(double right) {
        this.right = right;
        setValues(calculateFromWheelMovement(left, right, trackWidth));
    }

    public double getCurvature() {
        return curvature;
    }

    public double getDrivingRadius() {
        return 1 / curvature;
    }

    public double getTrackWidth() {
        return trackWidth;
    }
}
