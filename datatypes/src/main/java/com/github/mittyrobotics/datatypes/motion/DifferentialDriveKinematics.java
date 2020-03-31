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

/**
 * Contains differential drive kinematics equations for figuring out wheel velocities.
 * <p>
 * http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 */
public class DifferentialDriveKinematics {
    /**
     * Calculates the {@link DrivetrainWheelVelocities} given a base robot velocity and a radius of the circle that it wants
     * to follow using differential drive kinematics.
     *
     * @param linearVelocity the base linear robot velocity in units per second (how fast the robot moves forward).
     * @param radius         the radius to follow in units.
     * @param trackWidth     the width between left and right wheels of the drivetrain.
     * @return the calculated {@link DrivetrainWheelVelocities}.
     */
    public static DrivetrainWheelVelocities calculateFromRadius(double linearVelocity, double radius,
                                                                double trackWidth) {
        //Calculate the angular velocity of the robot in radians per second
        double angularVelocity = linearVelocity / radius;

        //Calculate left and right drivetrain velocities
        double left = angularVelocity * (radius - (trackWidth / 2));
        double right = angularVelocity * (radius + (trackWidth / 2));
        //left/(linearVelocity / angularVelocity - (track/2)) = angular
        //left/(linear/angular) - left/(track/2)
        //Return the calculated drivetrain velocities
        return new DrivetrainWheelVelocities(left, right);
    }

    /**
     * Calculates the {@link DrivetrainWheelVelocities} given a base robot velocity and a radius of the circle that it wants
     * to follow using differential drive kinematics.
     *
     * @param linearVelocity  the base linear robot velocity in units per second (how fast the robot moves forward).
     * @param angularVelocity the angular velocity of the robot in radians per second
     * @param trackWidth      the width between left and right wheels of the drivetrain.
     * @return the calculated {@link DrivetrainWheelVelocities}.
     */
    public static DrivetrainWheelVelocities calculateFromAngularVelocity(double linearVelocity, double angularVelocity,
                                                                  double trackWidth) {
        if (linearVelocity == 0 && angularVelocity == 0) {
            return new DrivetrainWheelVelocities(0, 0);
        }

        //Calculate the radius given linear velocity and angular velocity
        double radius = linearVelocity / angularVelocity;

        //Return the calculated drivetrain velocities
        return new DrivetrainWheelVelocities(angularVelocity * (radius - (trackWidth / 2)),
                angularVelocity * (radius + (trackWidth / 2)));
    }

    public static double getRadiusFromWheelSpeeds(DrivetrainWheelVelocities wheelSpeeds, double trackWidth) {
        double linearVelocity = wheelSpeeds.getAvgVelocity();

        return linearVelocity / getAngularVelocityFromWheelSpeeds(wheelSpeeds, trackWidth);
    }

    public static double getAngularVelocityFromWheelSpeeds(DrivetrainWheelVelocities wheelSpeeds, double trackWidth) {
        double rightVelocity = wheelSpeeds.getRightVelocity();
        double linearVelocity = wheelSpeeds.getAvgVelocity();


        double angularVelocity = (2 * (rightVelocity - linearVelocity)) / trackWidth;

        return angularVelocity;
    }
}