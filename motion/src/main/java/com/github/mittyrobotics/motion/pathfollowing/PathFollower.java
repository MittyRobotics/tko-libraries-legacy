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

import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithParameter;
import com.github.mittyrobotics.motion.pathfollowing.enums.PathFollowingType;
import com.github.mittyrobotics.path.generation.Path;
import com.github.mittyrobotics.path.generation.PathGenerator;

public abstract class PathFollower {
    private PathFollowingType pathFollowingType;

    private PathFollowerProperties properties;

    private double previousCalculatedVelocity = 0;
    private Path currentPath;
    private boolean unAdaptedPath;
    private double currentDistanceToEnd = Double.POSITIVE_INFINITY;




    private TransformWithParameter expectedPathTransform;

    /**
     * Constructs a {@link PathFollower}.
     *
     * @param properties the {@link PathFollowerProperties} for the {@link PathFollower}.
     */
    public PathFollower(PathFollowerProperties properties) {
        this.properties = properties;
    }

    /**
     * Sets the {@link Path} that the {@link PathFollower} is currently following.
     * <p>
     * This can be done at any time, even in the middle of following another {@link Path}. The {@link PathFollower}
     * will
     * automatically adapt the the new {@link Path} to the robot's next position when following.
     *
     * @param newPath the new {@link Path} to follow.
     */
    public void setPath(Path newPath) {
        setPath(newPath, false);
    }

    /**
     * Sets the {@link Path} that the {@link PathFollower} is currently following.
     * <p>
     * This can be done at any time, even in the middle of following another {@link Path}. This can be done at any
     * time, even in the middle of following another {@link Path}. The {@link PathFollower} will automatically
     * adapt the the new {@link Path} to the robot's next position when following if <code>adaptPathToRobot</code> is
     * true. Otherwise, if <code>adaptPathToRobot</code> is false, the robot will follow the input {@link Path}
     * unchanged.
     *
     * @param newPath          the new {@link Path} to follow.
     * @param adaptPathToRobot whether or not to adapt the {@link Path} passed in to the robot's location at the next
     *                         update function call.
     */
    public void setPath(Path newPath, boolean adaptPathToRobot) {
        this.currentPath = newPath;
        if (adaptPathToRobot) {
            unAdaptedPath = true;
        }
        else{
            expectedPathTransform =new TransformWithParameter(newPath.getStartWaypoint(), 0);
        }
        this.currentDistanceToEnd = 9999;
    }

    public void setDrivingGoal(Transform goal) {
        Path path =
                new Path(PathGenerator.generateQuinticHermiteSplinePath(new Transform[]{
                        goal,
                        goal}));
        setPath(path, true);
    }

    public void setDrivingGoalVia(Transform goal, Transform[] intermediatePoints) {
        Transform[] waypoints = new Transform[intermediatePoints.length + 1];
        for (int i = 0; i < waypoints.length + 1; i++) {
            waypoints[i] = intermediatePoints[i];
        }
        waypoints[waypoints.length - 1] = goal;
        Path path = new Path(PathGenerator.generateQuinticHermiteSplinePath(waypoints));
        setPath(path, true);
    }

    /**
     * Universal update function for the {@link PathFollower}.
     * <p>
     * Will update the respective path following controller depending on which {@link PathFollowerProperties} was
     * input.
     *
     * @param robotTransform              the robot's current {@link Transform}.
     * @param currentDrivetrainVelocities the robot's current {@link DrivetrainState}
     * @param deltaTime                   the change in time since the last update call in seconds.
     * @return the {@link DrivetrainState} to follow based on the path following algorithm.
     */
    public DrivetrainState updatePathFollower(Transform robotTransform,
                                              DrivetrainState currentDrivetrainVelocities,
                                              double deltaTime) {
        if (unAdaptedPath) {
            calculateAdaptivePath(robotTransform, currentDrivetrainVelocities.getCurvature());
            unAdaptedPath = false;
            expectedPathTransform = currentPath.getClosestTransform(robotTransform.getPosition());
        }

        if (currentPath == null) {
            System.out.println("WARNING: The current path follower path is null!");
            return DrivetrainState.empty();
        }

        //Find the rough distance to the end of the path
        this.currentDistanceToEnd = getRoughDistanceToEnd(robotTransform);

        DrivetrainState state = calculate(robotTransform, currentDrivetrainVelocities, deltaTime);

        double t = currentPath.getParameterFromLength(currentPath.getGaussianQuadratureLength(expectedPathTransform.getParameter()) + state.getLinear() * deltaTime);
        Transform oldTransform = expectedPathTransform;
        expectedPathTransform = new TransformWithParameter(new Transform(currentPath.getTransform(t)), t);
        return state;
    }

    public abstract DrivetrainState calculate(Transform robotTransform, DrivetrainState currentDrivetrainVelocities,
                                              double deltaTime);

    /**
     * Calculates an adapted {@link Path} from the robot's location to the current {@link Path}.
     *
     * @param robotTransform the robot's {@link Transform}.
     */
    private void calculateAdaptivePath(Transform robotTransform, double curvature) {
        Path path =
                new Path(PathGenerator.generateQuinticHermiteSplinePath(
                        currentPath.generateAdaptivePathWaypoints(robotTransform, true)));
        setPath(path);
    }

    /**
     * Returns whether the robot is within the <code>distanceTolerance</code> of the ending location of the {@link
     * Path}.
     *
     * @param distanceTolerance the distance threshold to end
     * @return whether the robot is within the <code>distanceTolerance</code> of the ending location of the {@link
     * Path}.
     */
    public boolean isFinished(double distanceTolerance) {
        return getCurrentDistanceToEnd() < distanceTolerance;
    }

    /**
     * Returns the rough distance of the robot along the current {@link Path}.
     *
     * @param robotTransform the robot's {@link Transform}.
     * @return the rough distance of the robot along the current {@link Path}.
     */
    private double getRoughDistanceToEnd(Transform robotTransform) {
        double closestParameter = currentPath.getClosestT(robotTransform.getPosition(), 10, 3);
        return currentPath.getGaussianQuadratureLength() - currentPath.getGaussianQuadratureLength(closestParameter);
    }

    /**
     * Returns the current {@link Path} being followed by the {@link PathFollower}.
     *
     * @return the current {@link Path} being followed by the {@link PathFollower}.
     */
    public Path getCurrentPath() {
        return currentPath;
    }

    public double getCurrentDistanceToEnd() {
        return currentDistanceToEnd;
    }

    public PathFollowerProperties getProperties() {
        return properties;
    }

    public double getPreviousCalculatedVelocity() {
        return previousCalculatedVelocity;
    }

    public void setPreviousCalculatedVelocity(double previousCalculatedVelocity) {
        this.previousCalculatedVelocity = previousCalculatedVelocity;
    }

    public TransformWithParameter getExpectedPathTransform() {
        return expectedPathTransform;
    }

    public void setExpectedPathTransform(TransformWithParameter expectedPathTransform) {
        this.expectedPathTransform = expectedPathTransform;
    }

}