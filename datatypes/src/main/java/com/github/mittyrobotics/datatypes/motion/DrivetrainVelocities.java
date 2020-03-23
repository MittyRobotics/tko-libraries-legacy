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
    private double linearVelocity;
    private double angularVelocity;
    private double leftVelocity;
    private double rightVelocity;
    private double drivingCurvature;

    private DrivetrainVelocities(double linearVelocity, double angularVelocity, double leftVelocity,
                                 double rightVelocity,
                                 double drivingCurvature) {
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.drivingCurvature = drivingCurvature;
    }

    public static DrivetrainVelocities empty() {
        return new DrivetrainVelocities(0, 0, 0, 0, 0);
    }

    public static DrivetrainVelocities calculateFromLinearVelocityAndRadius(double linearVelocity,
                                                                            double radius) {
        DrivetrainWheelVelocities drivetrainWheelVelocities =
                DifferentialDriveKinematics.getInstance().calculateFromRadius(linearVelocity, radius);
        return calculateFromWheelVelocities(drivetrainWheelVelocities);
    }

    public static DrivetrainVelocities calculateFromWheelVelocities(double leftVelocity, double rightVelocity) {
        return calculateFromWheelVelocities(new DrivetrainWheelVelocities(leftVelocity, rightVelocity));
    }

    public static DrivetrainVelocities calculateFromWheelVelocities(
            DrivetrainWheelVelocities drivetrainWheelVelocities) {
        //Get linear and angular velocity from drivetrain velocities
        double linearVelocity = drivetrainWheelVelocities.getAvgVelocity();
        double angularVelocity =
                DifferentialDriveKinematics.getInstance().getAngularVelocityFromWheelSpeeds(drivetrainWheelVelocities);

        //Get driving curvature from linear velocity and angular velocity
        double curvature = 1 / (linearVelocity / angularVelocity);

        if (Double.isNaN(curvature)) {
            curvature = 0;
        }

        return new DrivetrainVelocities(linearVelocity, angularVelocity, drivetrainWheelVelocities.getLeftVelocity(),
                drivetrainWheelVelocities.getRightVelocity(), curvature);
    }

    public static DrivetrainVelocities calculateFromLinearAndAngularVelocity(double linearVelocity,
                                                                             double angularVelocity) {
        //Calculate drivetrain velocities from linear and angular velocities
        DrivetrainWheelVelocities drivetrainWheelVelocities =
                DifferentialDriveKinematics.getInstance().calculateFromAngularVelocity(linearVelocity, angularVelocity);

        //Get left and right velocity from drivetrain velocities
        double leftVelocity = drivetrainWheelVelocities.getLeftVelocity();
        double rightVelocity = drivetrainWheelVelocities.getRightVelocity();

        //Get driving curvature from linear velocity and angular velocity
        double curvature = 1 / (linearVelocity / angularVelocity);

        return new DrivetrainVelocities(linearVelocity, angularVelocity, leftVelocity, rightVelocity, curvature);
    }

    public DrivetrainVelocities reverse() {
        return new DrivetrainVelocities(-linearVelocity, angularVelocity, -leftVelocity, -rightVelocity,
                drivingCurvature);
    }

    private void setValues(DrivetrainVelocities data) {
        this.linearVelocity = data.getLinearVelocity();
        this.angularVelocity = data.getAngularVelocity();
        this.leftVelocity = data.getLeftVelocity();
        this.rightVelocity = data.getRightVelocity();
        this.drivingCurvature = data.getDrivingCurvature();
    }

    public double getLinearVelocity() {
        return linearVelocity;
    }

    public void setLinearVelocity(double linearVelocity) {
        this.linearVelocity = linearVelocity;
        setValues(calculateFromLinearAndAngularVelocity(linearVelocity, angularVelocity));
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
        setValues(calculateFromLinearAndAngularVelocity(linearVelocity, angularVelocity));
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public void setLeftVelocity(double leftVelocity) {
        this.leftVelocity = leftVelocity;
        setValues(calculateFromWheelVelocities(leftVelocity, rightVelocity));
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public void setRightVelocity(double rightVelocity) {
        this.rightVelocity = rightVelocity;
        setValues(calculateFromWheelVelocities(leftVelocity, rightVelocity));
    }

    public double getDrivingCurvature() {
        return drivingCurvature;
    }

    public double getDrivingRadius() {
        return 1 / drivingCurvature;
    }
}
