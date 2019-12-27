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

package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithT;
import com.github.mittyrobotics.path.following.controllers.PurePursuitController;
import com.github.mittyrobotics.path.following.controllers.RamseteController;
import com.github.mittyrobotics.path.following.enums.PathFollowingType;
import com.github.mittyrobotics.path.following.util.PathFollowerProperties;
import com.github.mittyrobotics.path.generation.paths.Path;

public class PathFollower {
    private PathFollowingType pathFollowingType;

    private PathFollowerProperties properties;
    private PathFollowerProperties.PurePursuitProperties purePursuitProperties;
    private PathFollowerProperties.RamseteProperties ramseteProperties;

    private Path currentPath;

    private boolean unAdaptedPath;

    /**
     * Constructs a {@link PathFollower} and sets it up for use with the {@link PurePursuitController}.
     *
     * @param properties the {@link PathFollowerProperties.PurePursuitProperties} for the {@link PathFollower}.
     */
    public PathFollower(PathFollowerProperties.PurePursuitProperties properties) {
        setupPurePursuit(properties);
    }

    /**
     * Constructs a {@link PathFollower} and sets it up for use with the {@link RamseteController}.
     *
     * @param properties the {@link PathFollowerProperties.RamseteProperties} for the {@link PathFollower}.
     */
    public PathFollower(PathFollowerProperties.RamseteProperties properties) {
        setupRamseteController(properties);
    }

    /**
     * Sets up the {@link PurePursuitController} with the {@link PathFollowerProperties.PurePursuitProperties}.
     *
     * @param properties the {@link PathFollowerProperties.PurePursuitProperties} for the {@link
     *                   PurePursuitController}.
     */
    private void setupPurePursuit(PathFollowerProperties.PurePursuitProperties properties) {
        pathFollowingType = PathFollowingType.PURE_PURSUIT_CONTROLLER;

        setupPathFollower(properties);
        this.purePursuitProperties = properties;

        if (purePursuitProperties.curvatureSlowdownGain != -1) {
            PurePursuitController.getInstance().setCurvatureSlowdownGain(purePursuitProperties.curvatureSlowdownGain);
        }
        if (purePursuitProperties.minSlowdownVelocity != -1) {
            PurePursuitController.getInstance().setMinSlowdownVelocity(purePursuitProperties.minSlowdownVelocity);
        }

    }

    /**
     * Sets up the {@link RamseteController} with the {@link PathFollowerProperties.RamseteProperties}.
     *
     * @param properties the {@link PathFollowerProperties.RamseteProperties} for the {@link RamseteController}.
     */
    private void setupRamseteController(PathFollowerProperties.RamseteProperties properties) {
        this.pathFollowingType = PathFollowingType.RAMSETE_CONTROLLER;

        setupPathFollower(properties);
        this.ramseteProperties = properties;

        if (ramseteProperties.aggressiveGain != -1) {
            RamseteController.getInstance().setAggressiveGain(ramseteProperties.aggressiveGain);
        }
        if (ramseteProperties.dampingGain != -1) {
            RamseteController.getInstance().setDampingGain(ramseteProperties.dampingGain);
        }
    }

    /**
     * Universal setup function for all paths. Sets up the {@link PathFollower} with the {@link
     * PathFollowerProperties}.
     *
     * @param properties
     */
    private void setupPathFollower(PathFollowerProperties properties) {
        this.properties = properties;
        this.currentPath = properties.path;
    }

    /**
     * Changes the {@link Path} that the {@link PathFollower} is currently following.
     * <p>
     * This can be done at any time, even in the middle of following another {@link Path}. The {@link PathFollower}
     * will
     * automatically adapt the the new {@link Path} to the robot's next position when following.
     *
     * @param newPath the new {@link Path} to follow.
     */
    public void changePath(Path newPath) {
        this.currentPath = newPath;
        unAdaptedPath = true;
    }

    /**
     * Changes the {@link Path} that the {@link PathFollower} is currently following.
     * <p>
     * This can be done at any time, even in the middle of following another {@link Path}. This can be done at any
     * time,
     * even in the middle of following another {@link Path}. The {@link PathFollower} will * automatically adapt the
     * the
     * new {@link Path} to the robot's next position when following if <code>adaptPathToRobot</code> is true.
     * Otherwise,
     * if <code>adaptPathToRobot</code> is false, the robot will follow the input {@link Path} unchanged.
     *
     * @param newPath          the new {@link Path} to follow.
     * @param adaptPathToRobot whether or not to adapt the {@link Path} passed in to the robot's location at the next
     *                         update function call.
     */
    public void changePath(Path newPath, boolean adaptPathToRobot) {
        this.currentPath = newPath;
        if (adaptPathToRobot) {
            unAdaptedPath = true;
        }
    }

    /**
     * Universal update function for the {@link PathFollower}.
     * <p>
     * Will update the respective path following controller depending on which {@link PathFollowerProperties} was
     * input.
     *
     * @param robotTransform  the robot's current {@link Transform}.
     * @param currentVelocity the robot's current velocity in inches/s.
     * @param deltaTime       the change in time since the last update call in seconds.
     * @return the {@link DrivetrainVelocities} to follow based on the path following algorithm.
     */
    public DrivetrainVelocities updatePathFollower(Transform robotTransform, double currentVelocity,
                                                   double deltaTime) {
        if (unAdaptedPath) {
            calculateAdaptivePath(robotTransform, true);
            unAdaptedPath = false;
        } else if (properties.continuouslyAdaptivePath) {
            calculateAdaptivePath(robotTransform, false);
        }

        if (currentPath == null) {
            System.out.println("WARNING: The current path follower path is null!");
            return new DrivetrainVelocities(0, 0);
        }
        if (pathFollowingType == PathFollowingType.PURE_PURSUIT_CONTROLLER) {
            return updatePurePursuit(robotTransform, currentVelocity, deltaTime);
        } else if (pathFollowingType == PathFollowingType.RAMSETE_CONTROLLER) {
            return updateRamsete(robotTransform, currentVelocity, deltaTime);
        } else {
            System.out.println("WARNING: Unspecified path follower type");
            return new DrivetrainVelocities(0, 0);
        }
    }

    /**
     * Updates the {@link PurePursuitController} path following algorithm.
     *
     * @param robotTransform  the robot's current {@link Transform}.
     * @param currentVelocity the robot's current velocity in inches/s.
     * @param deltaTime       the change in time since the last update call in seconds.
     * @return the {@link DrivetrainVelocities} to follow based on the {@link PurePursuitController} algorithm.
     */
    private DrivetrainVelocities updatePurePursuit(Transform robotTransform, double currentVelocity,
                                                   double deltaTime) {
        double lookaheadDistance = purePursuitProperties.lookaheadDistance;

        Position lookaheadCalculationStartPosition;

        if (purePursuitProperties.adaptiveLookahead) {
            Position closestPosition = currentPath.getClosestTransform(robotTransform.getPosition()).getPosition();
            lookaheadCalculationStartPosition = closestPosition;
        } else {
            lookaheadCalculationStartPosition = robotTransform.getPosition();
        }

        Position targetPosition = currentPath.getClosestTransform(lookaheadCalculationStartPosition,
                lookaheadDistance).getPosition();

        //Find the rough distance to the end of the path
        double distanceToEnd = getDistanceToEnd(robotTransform, 20);

        //Calculate the robot velocity using the path velocity controller
        double robotVelocity = properties.velocityController.getVelocity(Math.abs(currentVelocity), distanceToEnd,
                deltaTime) * (properties.reversed ? -1 : 1);

        //Calculate the pure pursuit controller
        return PurePursuitController.getInstance().calculate(robotTransform, targetPosition, robotVelocity);
    }

    /**
     * Updates the {@link RamseteController} path following algorithm.
     *
     * @param robotTransform  the robot's current {@link Transform}.
     * @param currentVelocity the robot's current velocity in inches/s.
     * @param deltaTime       the change in time since the last update call in seconds.
     * @return the {@link DrivetrainVelocities} to follow based on the {@link RamseteController} algorithm.
     */
    private DrivetrainVelocities updateRamsete(Transform robotTransform, double currentVelocity, double deltaTime) {
        //Get the desired transform to follow, which is the closest point on the path
        TransformWithT desiredTransform = currentPath.getClosestTransform(robotTransform.getPosition());

        //If reversed, reverse the desired transform's rotation
        desiredTransform.setRotation(desiredTransform.getRotation().rotateBy(new Rotation((properties.reversed ? 180 :
                0))));

        //Find the rough distance to the end of the path
        double distanceToEnd = getDistanceToEnd(robotTransform, 20);

        //Calculate the robot velocity using the path velocity controller. If reversed, reverse the robot velocity
        double robotVelocity = properties.velocityController.getVelocity(Math.abs(currentVelocity), distanceToEnd,
                deltaTime)
                * (properties.reversed ? -1 : 1);

        //Get radius from curvature is 1/curvature
        double turningRadius = 1 / currentPath.getCurvature(desiredTransform.getT());

        if (Double.isNaN(turningRadius) || Double.isInfinite(turningRadius)) {
            turningRadius = 2e16;
        }

        return RamseteController.getInstance().calculate(robotTransform, desiredTransform, robotVelocity,
                turningRadius);
    }

    /**
     * Calculates an adapted {@link Path} from the robot's location to the current {@link Path}.
     *
     * @param robotTransform the robot's {@link Transform}.
     */
    private void calculateAdaptivePath(Transform robotTransform, boolean adaptToRobotHeading) {
        changePath(currentPath.updatePathFromPoints(currentPath.generateAdaptivePathWaypoints(robotTransform,
                adaptToRobotHeading)), false);
    }

    /**
     * Returns whether the robot is within the <code>distanceTolerance</code> of the ending location of the {@link
     * Path}.
     *
     * @param robotTransform    the robot's {@link Transform}.
     * @param distanceTolerance the distance threshold to end
     * @return whether the robot is within the <code>distanceTolerance</code> of the ending location of the {@link
     * Path}.
     */
    public boolean isFinished(Transform robotTransform, double distanceTolerance) {
        return getDistanceToEnd(robotTransform, 0) < distanceTolerance;
    }

    /**
     * Returns the distance from the robot to the end of the path.
     *
     * @param robotTransform        the robot's {@link Transform}.
     * @param stopDistanceTolerance
     * @return the distance from the robot to the end of the path.
     */
    private double getDistanceToEnd(Transform robotTransform, double stopDistanceTolerance) {
        double distance = robotTransform.getPosition().distance(currentPath.getEndWaypoint().getPosition());

        if (robotTransform.relativeTo(currentPath.getEndWaypoint()).getPosition().getX() >= 0 &&
                distance <= stopDistanceTolerance && stopDistanceTolerance != 0) {
            return 0;
        }

        return distance;
    }

    /**
     * Returns the current {@link Path} being followed by the {@link PathFollower}.
     *
     * @return the current {@link Path} being followed by the {@link PathFollower}.
     */
    public Path getCurrentPath() {
        return currentPath;
    }

}
