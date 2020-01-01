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
    private static Odometry instance;

    private Transform robotTransform;

    private double lastLeftEncoderPos = 0;
    private double lastRightEncoderPos = 0;
    private double calibrateGyroVal = 0;

    public static Odometry getInstance() {
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

    public void calibrateToZero(double leftEncoder, double rightEncoder, double heading) {
        calibrateGyroVal = heading;
        lastLeftEncoderPos = leftEncoder;
        lastRightEncoderPos = rightEncoder;
        setRobotTransform(new Transform(0, 0, 0));
    }

    public void calibratePositionToZero(double leftEncoder, double rightEncoder) {
        lastLeftEncoderPos = leftEncoder;
        lastRightEncoderPos = rightEncoder;
        setRobotTransform(new Transform(0, 0, robotTransform.getRotation()));
    }

    /**
     * Returns the calculated robot {@link Transform} from the {@link Odometry} calculations.
     *
     * @return the robot {@link Transform}.
     */
    public Transform getRobotTransform() {
        return robotTransform;
    }

    /**
     * Sets the robot {@link Transform}.
     *
     * @param newRobotTransform the new robot {@link Transform}.
     */
    public void setRobotTransform(Transform newRobotTransform) {
        this.robotTransform = newRobotTransform;
    }

    /**
     * Sets the robot {@link Position} in the robot {@link Transform}.
     *
     * @param newRobotPosition the new robot {@link Position}.
     */
    public void setRobotPosition(Position newRobotPosition) {
        this.robotTransform = new Transform(newRobotPosition, robotTransform.getRotation());
    }

    /**
     * Sets the robot {@link Transform} and calibrates the gyro value.
     *
     * @param newRobotTransform   the new robot {@link Transform}.
     * @param currentRobotHeading the robot's current heading value.
     */
    public void setRobotTransform(Transform newRobotTransform, double currentRobotHeading) {
        calibrateGyroValue(newRobotTransform.getRotation().getHeading(), currentRobotHeading);
        this.robotTransform = newRobotTransform;
    }

    public void calibrateGyroValue(double desiredHeading, double currentHeading) {
        this.calibrateGyroVal =
                currentHeading - desiredHeading;
    }
}
