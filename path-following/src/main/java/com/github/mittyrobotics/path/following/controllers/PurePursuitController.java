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

package com.github.mittyrobotics.path.following.controllers;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DifferentialDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.DrivetrainData;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class PurePursuitController {
    public static final double DEFAULT_CURVATURE_SLOWDOWN_GAIN = 0.0;
    public static final double DEFAULT_MIN_SLOWDOWN_VELOCITY = 20.0;
    public static double DEFAULT_LOOKAHEAD_DISTANCE = 25.0;
    private static PurePursuitController instance = new PurePursuitController();
    private double curvatureSlowdownGain;
    private double minSlowdownVelocity;

    private PurePursuitController() {

    }

    public static PurePursuitController getInstance() {
        return instance;
    }

    /**
     * Sets the gains for the {@link PurePursuitController}.
     *
     * @param curvatureSlowdownGain (x > 0), the gain to slow down the robot when it turns at a sharper curvature.
     *                              Smaller values make it go slower.
     * @param minSlowdownVelocity   (x > 0), the minimum velocity that the robot is allowed to slow down to at sharp
     *                              curvature.
     */
    public void setGains(double curvatureSlowdownGain, double minSlowdownVelocity) {
        this.curvatureSlowdownGain = curvatureSlowdownGain;
        this.minSlowdownVelocity = minSlowdownVelocity;
    }

    /**
     * Calculates the {@link DrivetrainVelocities} based on the {@link PurePursuitController} path following algorithm.
     *
     * @param robotTransform the robot's current {@link Transform}.
     * @param targetPosition the {@link Position} in front of the robot it is targeting (the look ahead position).
     * @param robotVelocity  the desired base velocity for the robot to be going.
     * @return the {@link DrivetrainVelocities} based on the {@link PurePursuitController} path following algorithm.
     */
    public DrivetrainData calculate(Transform robotTransform, Position targetPosition, double robotVelocity) {
        //Determine if reversed
        boolean reversed = robotVelocity < 0;

        //If reversed, flip the robot's transform
        if (reversed) {
            robotTransform.setRotation(robotTransform.getRotation().add(new Rotation(180)));
        }

        //Calculate the pursuit circle to follow, calculated by finding the circle tangent to the robot transform that
        //intersects the target position.
        Circle pursuitCircle = new Circle(robotTransform, targetPosition);

        //Determine which side the robot transform is on the circle
        double side = new Line(robotTransform.getPosition(),
                robotTransform.getPosition().add(
                        new Position(
                                robotTransform.getRotation().cos() * 5,
                                robotTransform.getRotation().sin() * 5)
                )).findSide(pursuitCircle.getCenter());

        robotVelocity = calculateSlowdownVelocity(1 / (pursuitCircle.getRadius()), robotVelocity, minSlowdownVelocity);

        double radius = pursuitCircle.getRadius() * side * (reversed ? -1 : 1);

        //Use differential drive kinematics to calculate the left and right wheel velocity given the base robot
        //velocity and the radius of the pursuit circle
        return new DrivetrainData(DifferentialDriveKinematics.getInstance().calculateFromRadius(
                robotVelocity, radius));
    }

    /**
     * Calculates the velocity of the robot after applying the <code>curvatureSlowdownGain</code> to the
     * <code>curvature</code>.
     *
     * @param curvature           the curvature (1/radius) of the arc the robot is following.
     * @param currentVelocity     the current velocity that the robot is or should be moving at.
     * @param minSlowdownVelocity the minimum allowed velocity to slow down to.
     * @return the velocity of the robot after applying the <code>curvatureSlowdownGain</code> to the
     * <code>curvature</code>.
     */
    private double calculateSlowdownVelocity(double curvature, double currentVelocity, double minSlowdownVelocity) {
        if (curvatureSlowdownGain == 0) {
            return currentVelocity;
        }
        double absVelocity = Math.abs(currentVelocity);
        double velSign = Math.signum(currentVelocity);
        double vel = Math.min(absVelocity, Math.max(minSlowdownVelocity, curvatureSlowdownGain / curvature));
        return vel * velSign;
    }

    public double getCurvatureSlowdownGain() {
        return curvatureSlowdownGain;
    }

    public void setCurvatureSlowdownGain(double curvatureSlowdownGain) {
        this.curvatureSlowdownGain = curvatureSlowdownGain;
    }

    public double getMinSlowdownVelocity() {
        return minSlowdownVelocity;
    }

    public void setMinSlowdownVelocity(double minSlowdownVelocity) {
        this.minSlowdownVelocity = minSlowdownVelocity;
    }
}
