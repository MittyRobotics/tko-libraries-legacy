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

import com.github.mittyrobotics.datatypes.CircularTimestampedList;
import com.github.mittyrobotics.datatypes.TimestampedElement;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class Odometry {
    private static Odometry instance = new Odometry();

    private CircularTimestampedList<Transform> robotTransformList = new CircularTimestampedList<>(100);
    private CircularTimestampedList<Transform> robotVelocityList = new CircularTimestampedList<>(100);

    private double lastLeftEncoderPos = 0;
    private double lastRightEncoderPos = 0;
    private double calibrateGyroVal = 0;

    public static Odometry getInstance() {
        return instance;
    }

    /**
     * Updates the {@link Odometry}. This should be updated frequently with the current encoder and gyro values.
     *
     * @param leftEncoderPosInches  The left wheel encoder value of the drivetrain in inches.
     * @param rightEncoderPosInches The right wheel encoder value of the drivetrain in inches.
     * @param gyro                  The robot gyroscope value.
     * @param timestamp             The timestamp of the update call. The easiest way to get the current
     *                              timestamp is to use WPILib's {@link edu.wpi.first.wpilibj.Timer}
     *                              <code>getFPGATimestamp()</code> method.
     */
    public void update(double leftEncoderPosInches, double rightEncoderPosInches, double gyro, double timestamp) {
        //Get robot rotation
        Rotation robotRotation = new Rotation(gyro - calibrateGyroVal).mapHeading180();

        //Get delta left and right encoder pos
        double deltaLeftPos = leftEncoderPosInches - lastLeftEncoderPos;
        double deltaRightPos = rightEncoderPosInches - lastRightEncoderPos;

        //Get average delta encoder pos in inches
        double deltaEncoder = (deltaLeftPos + deltaRightPos) / 2;

        //Get x and y values from heading and delta pos
        double deltaX = deltaEncoder * robotRotation.cos();
        double deltaY = deltaEncoder * robotRotation.sin();

        //Set last encoder positions
        lastLeftEncoderPos = leftEncoderPosInches;
        lastRightEncoderPos = rightEncoderPosInches;

        //Get delta position
        Position deltaPosition = new Position(deltaX, deltaY);
        //Get latest transform from timestamped transform list
        Transform latestTransform = robotTransformList.getLatest().getObject();
        //Get new transform from delta position and last transform
        Transform transform = new Transform(latestTransform.getPosition().add(deltaPosition), robotRotation);

        //Get delta time of update from timestamp list
        double deltaTime = timestamp - robotTransformList.getLatest().getTimestamp();
        //Compute acceleration transform from the difference between the current transform and last transform
        //multiplied by the delta time
        Transform velocity = transform.subtract(latestTransform).divide(deltaTime);

        //Add transform and acceleration to timestamped list
        robotTransformList.addFront(new TimestampedElement<>(transform, timestamp));
        robotVelocityList.addFront(new TimestampedElement<>(velocity, timestamp));
    }

    /**
     * Sets the heading of {@link Odometry}.
     * <p>
     * Calibrates the {@link Odometry} heading such that a gyro value on the robot of <code>gyro</code> will equal a
     * heading value in {@link Odometry} of <code>heading</code>.
     * <p>
     * For example, if your robot's gyro reads 30 degrees, and you want the {@link Odometry} heading to be 0, you
     * input a <code>heading</code> value of 0 and a <code>gyro</code> value of 30. From then on, the
     * {@link Odometry} heading value will be zeroed at the robot's gyro value of 30 degrees.
     *
     * @param heading the desired {@link Odometry} heading.
     * @param gyro    the robot's current gyroscope value.
     */
    public void setHeading(double heading, double gyro) {
        this.calibrateGyroVal = gyro - heading;
    }

    /**
     * Sets the {@link Odometry}'s position. Also calibrates all {@link Transform}s in the
     * <code>robotTransformList</code> to be as if the latest {@link Transform} has a {@link Position} of
     * <code>position</code> and still follows the <code>robotVelocityList</code> velocities.
     *
     * @param position the {@link Position} to set {@link Odometry} to.
     */
    public void setPosition(Position position) {
        robotTransformList.setObject(0, new Transform(position, robotTransformList.getLatest().getTimestamp()));
        //Loop through transform list, updating each transform based on the previous transform and velocity between
        //current transform and previous transform
        for (int i = 1; i < robotTransformList.size(); i++) {
            //Get delta time between initial and final transforms
            double deltaTime = robotTransformList.get(i - 1).getTimestamp() - robotTransformList.get(i).getTimestamp();
            //Get velocity from initial to final transform, which is the velocity transform in the velocity list with
            //the same index as the final transform in the transform list
            Transform velocity = robotVelocityList.get(i - 1).getObject();
            //Get the final transform in the transform list
            Transform finalTransform = robotTransformList.get(i - 1).getObject();
            //Calculate the initial transform using the formula: Xi = v*-t+Xf, where Xi is initial transform, t is
            //delta time, v is velocity, and Xf is final transform
            Transform initialTransform = velocity.multiply(-deltaTime).add(finalTransform);
            //Set the object to the new calculated initial transform
            robotTransformList.setObject(i, initialTransform);
        }
    }

    /**
     * Sets the {@link Odometry}'s {@link Transform} to <code>transform</code>.
     *
     * @param transform the {@link Transform} to set {@link Odometry} to.
     */
    public void setTransform(Transform transform, double gyro){
        setHeading(transform.getRotation().getHeading(), gyro);
        setPosition(transform.getPosition());
    }

    public void calibrateEncodersToZero(double leftEncoder, double rightEncoder){
        this.lastLeftEncoderPos = leftEncoder;
        this.lastRightEncoderPos = rightEncoder;
    }

    public void calibrateTransformToZero(double leftEncoder, double rightEncoder, double gyro){
        calibrateEncodersToZero(leftEncoder,rightEncoder);
        setTransform(new Transform(0,0, 0),gyro);
    }

    public Transform getLatestRobotTransform(){
        return robotTransformList.getLatest().getObject();
    }

    public Transform getLatestRobotVelocity(){
        return robotVelocityList.getLatest().getObject();
    }

    public double getLatestTimestamp(){
        return robotTransformList.getLatest().getTimestamp();
    }

    public Transform getRobotTransformAtTimestamp(double timestamp){
        return robotTransformList.getElementFromTimestamp(timestamp);
    }

    public Transform getRobotVelocityAtTimestamp(double timestamp){
        return robotVelocityList.getElementFromTimestamp(timestamp);
    }

    public CircularTimestampedList<Transform> getRobotTransformList() {
        return robotTransformList;
    }

    public CircularTimestampedList<Transform> getRobotVelocityList() {
        return robotVelocityList;
    }
}