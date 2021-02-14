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

package com.github.mittyrobotics.motion.pathfollowing;

import com.github.mittyrobotics.datatypes.CircularTimestampedList;
import com.github.mittyrobotics.datatypes.TimestampedElement;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class TimedRobotTracker {
    private static TimedRobotTracker instance = new TimedRobotTracker();

    private final CircularTimestampedList<Transform> robotTransformList;
    private final CircularTimestampedList<Transform> robotVelocityList;

    private Position latestCalibrationPosition;

    private final Odometry odometry;

    private TimedRobotTracker() {
        //Init latest calibration position
        latestCalibrationPosition = new Position();

        //Init circular timestamped lists
        robotTransformList = new CircularTimestampedList<>(100);
        robotVelocityList = new CircularTimestampedList<>(100);

        //Add first element to list to avoid null errors with update()
        robotTransformList.addFront(new TimestampedElement<>(new Transform(), 0));
        robotVelocityList.addFront(new TimestampedElement<>(new Transform(), 0));

        odometry = new Odometry();
    }

    public static TimedRobotTracker getInstance() {
        return instance;
    }

    /**
     * Updates the {@link TimedRobotTracker}. This should be updated frequently with the current encoder and gyro values.
     *
     * @param leftEncoderPosInches  The left wheel encoder value of the drivetrain in inches.
     * @param rightEncoderPosInches The right wheel encoder value of the drivetrain in inches.
     * @param gyro                  The robot gyroscope value.
     * @param timestamp             The timestamp of the update call. The easiest way to get the current
     *                              timestamp is to use WPILib's {@link edu.wpi.first.wpilibj.Timer}
     *                              <code>getFPGATimestamp()</code> method.
     */
    public void update(double leftEncoderPosInches, double rightEncoderPosInches, double gyro, double timestamp) {
        odometry.update(leftEncoderPosInches, rightEncoderPosInches, gyro);
        Position deltaPosition = odometry.getDeltaPosition();
        Rotation robotRotation = odometry.getRobotRotation();
        //Get latest transform from timestamped transform list
        Transform latestTransform = robotTransformList.getLatest().getObject();
        //Get new transform from delta position and last calibration position
        Transform transform = new Transform(latestCalibrationPosition.add(deltaPosition), robotRotation);

        //Get delta time of update from timestamp list
        double deltaTime = timestamp - robotTransformList.getLatest().getTimestamp();
        //Compute acceleration transform from the difference between the current transform and last transform
        //multiplied by the delta time
        Transform velocity = transform.subtract(latestTransform).divide(deltaTime);

        //Add transform and acceleration to timestamped list
        this.latestCalibrationPosition = transform.getPosition();
        robotTransformList.addFront(new TimestampedElement<>(transform, timestamp));
        robotVelocityList.addFront(new TimestampedElement<>(velocity, timestamp));
    }

    /**
     * Sets the heading of {@link TimedRobotTracker}.
     * <p>
     * Calibrates the {@link TimedRobotTracker} heading such that a gyro value on the robot of <code>gyro</code> will equal a
     * heading value in {@link TimedRobotTracker} of <code>heading</code>.
     * <p>
     * For example, if your robot's gyro reads 30 degrees, and you want the {@link TimedRobotTracker} heading to be 0, you
     * input a <code>heading</code> value of 0 and a <code>gyro</code> value of 30. From then on, the
     * {@link TimedRobotTracker} heading value will be zeroed at the robot's gyro value of 30 degrees.
     *
     * @param heading the desired {@link TimedRobotTracker} heading.
     * @param gyro    the robot's current gyroscope value.
     */
    public void setHeading(double heading, double gyro) {
        odometry.setHeading(heading, gyro);
    }

    /**
     * Sets the {@link TimedRobotTracker}'s position. Also calibrates all {@link Transform}s in the
     * <code>robotTransformList</code> to be as if the latest {@link Transform} has a {@link Position} of
     * <code>position</code> and still follows the <code>robotVelocityList</code> velocities.
     *
     * @param position the {@link Position} to set {@link TimedRobotTracker} to.
     */
    public void setPosition(Position position) {
        this.latestCalibrationPosition = position;
    }

    /**
     * Sets the {@link TimedRobotTracker}'s {@link Transform} to <code>transform</code>.
     *
     * @param transform the {@link Transform} to set {@link TimedRobotTracker} to.
     */
    public void setTransform(Transform transform, double gyro) {
        setHeading(transform.getRotation().getDegrees(), gyro);
        setPosition(transform.getPosition());
    }

    public void zeroEncoders(double leftEncoder, double rightEncoder) {
        odometry.zeroEncoders(leftEncoder, rightEncoder);
    }

    public void calibrateTransformToZero(double leftEncoder, double rightEncoder, double gyro) {
        zeroEncoders(leftEncoder, rightEncoder);
        setTransform(new Transform(0, 0, 0), gyro);
    }

    public Transform getLatestRobotTransform() {
        return robotTransformList.getLatest().getObject();
    }

    public Transform getLatestRobotVelocity() {
        return robotVelocityList.getLatest().getObject();
    }

    public double getLatestTimestamp() {
        return robotTransformList.getLatest().getTimestamp();
    }

    public Transform getRobotTransformAtTimestamp(double timestamp) {
        return robotTransformList.getElementFromTimestamp(timestamp);
    }

    public Transform getRobotVelocityAtTimestamp(double timestamp) {
        return robotVelocityList.getElementFromTimestamp(timestamp);
    }

    public CircularTimestampedList<Transform> getRobotTransformList() {
        return robotTransformList;
    }

    public CircularTimestampedList<Transform> getRobotVelocityList() {
        return robotVelocityList;
    }
}