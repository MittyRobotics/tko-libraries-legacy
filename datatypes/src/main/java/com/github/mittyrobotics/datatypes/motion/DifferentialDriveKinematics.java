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
     * Calculates the {@link DrivetrainWheelSpeeds} given a linear robot movement and a radius of the circle that it
     * wants to follow using differential drive kinematics.
     *
     * @param linear the linear robot movement in units per second (how fast the robot moves forward).
     * @param radius         the radius to follow in units.
     * @param trackWidth     the width between left and right wheels of the drivetrain.
     * @return the calculated {@link DrivetrainWheelSpeeds}.
     */
    public static DrivetrainWheelSpeeds calculateFromRadius(double linear, double radius,
                                                            double trackWidth) {
        //Calculate the angular velocity of the robot in radians per second
        double angular = linear / radius;

        //Calculate left and right drivetrain velocities
        double left = angular * (radius - (trackWidth / 2));
        double right = angular * (radius + (trackWidth / 2));
        //left/(linearVelocity / angular - (track/2)) = angular
        //left/(linear/angular) - left/(track/2)
        //Return the calculated drivetrain velocities
        return new DrivetrainWheelSpeeds(left, right);
    }

    /**
     * Calculates the {@link DrivetrainWheelSpeeds} given a robot linear and angular movement.
     *
     * @param linear  the linear robot movement in units per second (how fast the robot moves forward).
     * @param angular the angular movement of the robot in radians per second
     * @param trackWidth      the width between left and right wheels of the drivetrain.
     * @return the calculated {@link DrivetrainWheelSpeeds}.
     */
    public static DrivetrainWheelSpeeds calculateFromAngularMovement(double linear, double angular,
                                                                     double trackWidth) {
        if (linear == 0 && angular == 0) {
            return new DrivetrainWheelSpeeds(0, 0);
        }

        //Calculate the radius given linear velocity and angular velocity
        double radius = linear / angular;

        //Return the calculated drivetrain velocities
        return new DrivetrainWheelSpeeds(angular * (radius - (trackWidth / 2)),
                angular * (radius + (trackWidth / 2)));
    }

    public static double getRadiusFromWheelSpeeds(DrivetrainWheelSpeeds wheelSpeeds, double trackWidth) {
        double linearVelocity = wheelSpeeds.getAvgSpeed();

        return linearVelocity / getAngularVelocityFromWheelSpeeds(wheelSpeeds, trackWidth);
    }

    public static double getAngularVelocityFromWheelSpeeds(DrivetrainWheelSpeeds wheelSpeeds, double trackWidth) {
        double rightVelocity = wheelSpeeds.getRightSpeed();
        double linearVelocity = wheelSpeeds.getAvgSpeed();


        double angularVelocity = (2 * (rightVelocity - linearVelocity)) / trackWidth;

        return angularVelocity;
    }
}