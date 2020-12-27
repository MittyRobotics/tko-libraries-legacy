/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion.controllers;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithParameter;
import com.github.mittyrobotics.motion.pathfollowing.PathFollower;
import com.github.mittyrobotics.motion.pathfollowing.PathFollowerProperties;

public class PurePursuitController extends PathFollower {
    public static double DEFAULT_LOOKAHEAD_DISTANCE = 25.0;
    public static double DEFAULT_CURVATURE_SLOWDOWN_GAIN = 0.0;
    public static double DEFAULT_MIN_SLOWDOWN_VELOCITY = 0.0;

    private PathFollowerProperties.PurePursuitProperties purePursuitProperties;

    private Circle pursuitCircle;

    public PurePursuitController(PathFollowerProperties properties,
                                 PathFollowerProperties.PurePursuitProperties purePursuitProperties) {
        super(properties);
        this.purePursuitProperties = purePursuitProperties;
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
    private static double calculateSlowdownVelocity(double curvature, double curvatureSlowdownGain,
                                                    double currentVelocity,
                                                    double minSlowdownVelocity) {
        if (curvatureSlowdownGain == 0) {
            return currentVelocity;
        }
        double absVelocity = Math.abs(currentVelocity);
        double velSign = Math.signum(currentVelocity);
        double vel = Math.min(absVelocity, Math.max(minSlowdownVelocity, curvatureSlowdownGain / curvature));
        return vel * velSign;
    }

    @Override
    public DrivetrainState calculate(Transform robotTransform, DrivetrainState currentDrivetrainVelocities,
                                     double deltaTime) {
        double lookaheadDistance = purePursuitProperties.lookaheadDistance;

        TransformWithParameter closestTransformWithParameter =
                getCurrentPath().getClosestTransform(robotTransform.getPosition());

        Position targetPosition = getCurrentPath().getClosestTransform(closestTransformWithParameter.getPosition(),
                lookaheadDistance).getPosition();

        //Calculate the robot velocity using the path velocity controller
        double robotVelocity = getProperties().velocityController
                .getVelocity(getPreviousCalculatedVelocity(), getCurrentDistanceToEnd(),
                        deltaTime);

        setPreviousCalculatedVelocity(robotVelocity);

        //Calculate the pursuit circle to follow, calculated by finding the circle tangent to the robot transform that
        //intersects the target position.
        this.pursuitCircle = new Circle(robotTransform, targetPosition);

        //Determine which side the robot transform is on the circle
        double side = new Line(robotTransform.getPosition(),
                robotTransform.getPosition().add(
                        new Position(
                                robotTransform.getRotation().cos() * 5,
                                robotTransform.getRotation().sin() * 5)
                )).findSide(pursuitCircle.getCenter());

        double radius = pursuitCircle.getRadius() * side;

        robotVelocity =
                calculateSlowdownVelocity(1 / (pursuitCircle.getRadius()), purePursuitProperties.curvatureSlowdownGain,
                        robotVelocity,
                        purePursuitProperties.minSlowdownVelocity);

        //Use differential drive kinematics to calculate the left and right wheel velocity given the base robot
        //velocity and the radius of the pursuit circle
        DrivetrainState state =
                DrivetrainState.fromLinearAndRadius(robotVelocity, radius, getProperties().trackWidth);

        if (getProperties().reversed) {
            state = state.reverse();
        }

        return state;
    }

    public Circle getPursuitCircle() {
        return pursuitCircle;
    }
}
