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

/**
 * Contains differential drive kinematics equations for figuring out wheel velocities.
 * <p>
 * http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 */
public class DifferentialDriveKinematics {
    private static DifferentialDriveKinematics instance = new DifferentialDriveKinematics();

    private double trackWidth;

    private DifferentialDriveKinematics() {

    }

    public static DifferentialDriveKinematics getInstance() {
        return instance;
    }

    /**
     * Calculates the {@link DrivetrainVelocities} given a base robot velocity and a radius of the circle that it wants
     * to follow using differential drive kinematics.
     *
     * @param linearVelocity the base linear robot velocity in units per second (how fast the robot moves forward).
     * @param radius         the radius to follow in units.
     * @return the calculated {@link DrivetrainVelocities}.
     */
    public DrivetrainVelocities calculateFromRadius(double linearVelocity, double radius) {
        //Calculate the angular velocity of the robot in radians per second
        double angularVelocity = linearVelocity / radius;

        //If track width has not been set, send a warning and return velocities of 0 to avoid any damage
        if (trackWidth == 0) {
            System.out.println("WARNING: Track width in DifferentialDriveKinematics.java has not been set!");
            return new DrivetrainVelocities(0, 0);
        }

        //Calculate left and right drivetrain velocities
        double left = angularVelocity * (radius - (trackWidth / 2));
        double right = angularVelocity * (radius + (trackWidth / 2));
        //left/(linearVelocity / angularVelocity - (track/2)) = angular
        //left/(linear/angular) - left/(track/2)
        //Return the calculated drivetrain velocities
        return new DrivetrainVelocities(left, right);
    }

    /**
     * Calculates the {@link DrivetrainVelocities} given a base robot velocity and a radius of the circle that it wants
     * to follow using differential drive kinematics.
     *
     * @param linearVelocity  the base linear robot velocity in units per second (how fast the robot moves forward).
     * @param angularVelocity the angular velocity of the robot in radians per second
     * @return the calculated {@link DrivetrainVelocities}.
     */
    public DrivetrainVelocities calculateFromAngularVelocity(double linearVelocity, double angularVelocity) {
        if (linearVelocity == 0 && angularVelocity == 0) {
            return new DrivetrainVelocities(0, 0);
        }

        //Calculate the radius given linear velocity and angular velocity
        double radius = linearVelocity / angularVelocity;

        //If track width has not been set, send a warning and return velocities of 0 to avoid any damage
        if (trackWidth == 0) {
            System.out.println("WARNING: Track width in DifferentialDriveKinematics.java has not been set!");
            return new DrivetrainVelocities(0, 0);
        }

        //Return the calculated drivetrain velocities
        return new DrivetrainVelocities(angularVelocity * (radius - (trackWidth / 2)),
                angularVelocity * (radius + (trackWidth / 2)));
    }

    public double getRadiusFromWheelSpeeds(DrivetrainVelocities wheelSpeeds) {
        double linearVelocity = wheelSpeeds.getAvgVelocity();

        return linearVelocity / getAngularVelocityFromWheelSpeeds(wheelSpeeds);
    }

    public double getAngularVelocityFromWheelSpeeds(DrivetrainVelocities wheelSpeeds) {
        double rightVelocity = wheelSpeeds.getRightVelocity();
        double linearVelocity = wheelSpeeds.getAvgVelocity();

        //If track width has not been set, send a warning and return velocities of 0 to avoid any damage
        if (trackWidth == 0) {
            System.out.println("WARNING: Track width in DifferentialDriveKinematics.java has not been set!");
            return 2e16;
        }

        double angularVelocity = (2 * (rightVelocity - linearVelocity)) / trackWidth;

        return angularVelocity;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }
}