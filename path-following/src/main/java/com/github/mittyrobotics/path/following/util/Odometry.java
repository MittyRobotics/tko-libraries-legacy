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

package com.github.mittyrobotics.path.following.util;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class Odometry {
    private static Odometry instance = new Odometry();

    private Transform robotTransform;

    private double lastLeftEncoderPos = 0;
    private double lastRightEncoderPos = 0;
    private double calibrateGyroVal = 0;

    synchronized public static Odometry getInstance() {
        return instance;
    }

    /**
     * Updates the {@link Odometry}. This should be updated frequently with the current gencoder and gyro values.
     *
     * @param leftEncoderPosInches  the left wheel encoder value of the drivetrain in inches.
     * @param rightEncoderPosInches the right wheel encoder value of the drivetrain in inches.
     * @param heading               the heading value of the gyro.
     */
    public void update(double leftEncoderPosInches, double rightEncoderPosInches, double heading) {
        //Get robot rotation
        Rotation robotRotation = new Rotation(heading - calibrateGyroVal).mapHeading180();

        //Get delta left and right encoder pos
        double deltaLeftPos = leftEncoderPosInches - lastLeftEncoderPos;
        double deltaRightPos = rightEncoderPosInches - lastRightEncoderPos;

        //Get average delta encoder pos in inches
        double deltaPosition = (deltaLeftPos + deltaRightPos) / 2;

        //Get x and y values from heading and delta pos
        double deltaX = deltaPosition * robotRotation.cos();
        double deltaY = deltaPosition * robotRotation.sin();

        //Set last encoder positions
        lastLeftEncoderPos = leftEncoderPosInches;
        lastRightEncoderPos = rightEncoderPosInches;

        robotTransform =
                new Transform(robotTransform.getPosition().add(new Position(deltaX, deltaY)), robotRotation);

    }

    /**
     * Calibrates the {@link Odometry} robot {@link Transform} to the <code>newRobotTransform</code>.
     *
     * @param newRobotTransform the new {@link Transform} to set the {@link Odometry} to.
     * @param leftEncoder the current drivetrain left encoder position
     * @param rightEncoder the current drivetrain right encoder position
     * @param heading the current robot heading
     */
    public void calibrateRobotTransform(Transform newRobotTransform, double leftEncoder, double rightEncoder,
                                        double heading) {
        lastLeftEncoderPos = leftEncoder;
        lastRightEncoderPos = rightEncoder;
        calibrateGyroValue(newRobotTransform.getRotation().getHeading(), heading);
        this.robotTransform = newRobotTransform;
    }

    /**
     * Calibrates the {@link Odometry} robot {@link Transform} to zero.
     *
     * @param leftEncoder the current drivetrain left encoder position
     * @param rightEncoder the current drivetrain right encoder position
     * @param heading the current robot heading
     */
    public void calibrateTransformToZero(double leftEncoder, double rightEncoder, double heading) {
        calibrateRobotTransform(new Transform(0,0,0),leftEncoder,rightEncoder,heading);
    }

    /**
     * Calibrates the {@link Odometry} robot {@link Position} to zero while maintaining the current heading value.
     *
     * @param leftEncoder the current drivetrain left encoder position
     * @param rightEncoder the current drivetrain right encoder position
     */
    public void calibratePositionToZero(double leftEncoder, double rightEncoder) {
        lastLeftEncoderPos = leftEncoder;
        lastRightEncoderPos = rightEncoder;
        this.robotTransform = new Transform(0,0,getRobotTransform().getRotation());
    }

    /**
     * Returns the calculated robot {@link Transform} from the {@link Odometry} calculations.
     *
     * @return the robot {@link Transform}.
     */
    public Transform getRobotTransform() {
        return robotTransform;
    }

    public void calibrateGyroValue(double desiredHeading, double currentHeading) {
        this.calibrateGyroVal =
                currentHeading - desiredHeading;
    }
}
