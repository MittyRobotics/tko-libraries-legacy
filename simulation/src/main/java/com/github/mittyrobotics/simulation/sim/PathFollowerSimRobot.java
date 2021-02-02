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

package com.github.mittyrobotics.simulation.sim;

import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motion.controllers.PathVelocityController;
import com.github.mittyrobotics.motion.controllers.PurePursuitController;
import com.github.mittyrobotics.motion.pathfollowing.PathFollowerProperties;
import com.github.mittyrobotics.path.generation.Path;
import com.github.mittyrobotics.path.generation.PathGenerator;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphUtil;
import com.github.mittyrobotics.visualization.XYSeriesWithRenderer;

public class PathFollowerSimRobot extends SimRobot {
    private PurePursuitController follower;
    private Graph graph;
    private double time;
    private boolean updatedPath = false;

    public PathFollowerSimRobot(SimDrivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public void robotInit() {
        PathVelocityController velocityController = new PathVelocityController(1, 1, 50, 0, 0);
        double trackWidth = .5;
        boolean reversed = false;
        getDrivetrain().setupPIDFValues(0, 0, 0, 12.0 / 4.1190227085647875);
        follower =
                new PurePursuitController(new PathFollowerProperties(velocityController, trackWidth, reversed, false),
                        new PathFollowerProperties.PurePursuitProperties(.5, 0, 0));
        follower.setPath(new Path(PathGenerator.generateQuinticHermiteSplinePath(new Transform[]{new Transform(0, 0, 0), new Transform(4, 4, 0)})));
    }

    @Override
    public void robotPeriodic() {
        time += getRobotSimulator().getPeriodTime();
        DrivetrainState velocity = DrivetrainState
                .fromWheelSpeeds(getDrivetrain().getDrivetrainModel().getLeftVelocity(),
                        getDrivetrain().getDrivetrainModel().getRightVelocity(),
                        follower.getProperties().trackWidth);
        DrivetrainState newVelocity = follower.updatePathFollower(getDrivetrain().getRobotTransform(), velocity,
                getRobotSimulator().getPeriodTime());
        if (!updatedPath) {
            getRobotSimulator().getGraph().addToSeries("Path", GraphUtil
                    .populateSeries(new XYSeriesWithRenderer("Path"),
                            GraphUtil.parametric(follower.getCurrentPath(), 0.01, .1)));
            updatedPath = true;
        }
        getRobotSimulator().getGraph().changeSeries("Circle", GraphUtil
                .populateSeries(new XYSeriesWithRenderer("Circle"), GraphUtil.circle(follower.getPursuitCircle())));
//        Transform closestTransform = follower.getCurrentPath().getTransform(follower.getCurrentPath().getClosestT(getDrivetrain().getRobotTransform().getPosition(), 10, 10));
//        getRobotSimulator().getGraph().changeSeries("Point", GraphUtil.populateSeries(new XYSeriesWithRenderer("Point"), GraphUtil.arrow(closestTransform, .1, .1)));
//        follower.setPreviousTransformOnPath(closestTransform);
        getDrivetrain().setVelocityControl(newVelocity.getLeft(), newVelocity.getRight());
    }
}
