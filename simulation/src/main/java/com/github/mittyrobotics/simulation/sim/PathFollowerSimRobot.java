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

import com.github.mittyrobotics.datacollection.performance.TimeMonitor;
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
import org.jfree.data.xy.XYDataItem;

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
//        graph = new Graph();
        PathVelocityController velocityController = new PathVelocityController(.5, .5, 2, 0, 0, .4, .2);
        double trackWidth = .5;
        boolean reversed = true;
        getDrivetrain().setupPIDFValues(1, 0, .001, 12.0 / 4.1190227085647875);
        follower =
                new PurePursuitController(new PathFollowerProperties(velocityController, trackWidth, reversed, false),
                        new PathFollowerProperties.PurePursuitProperties(.5));
        follower.setPath(new Path(PathGenerator.generateQuinticHermiteSplinePath(new Transform[]{new Transform(0, 0, Math.PI), new Transform(-4, 0, Math.PI)})));

    }

    @Override
    public void robotPeriodic() {
        time += getRobotSimulator().getPeriodTime();
        DrivetrainState velocity = DrivetrainState
                .fromWheelSpeeds(getDrivetrain().getDrivetrainModel().getLeftVelocity(),
                        getDrivetrain().getDrivetrainModel().getRightVelocity(),
                        follower.getProperties().trackWidth);

        TimeMonitor monitor = new TimeMonitor();
        monitor.start();

        DrivetrainState newVelocity = follower.updatePathFollower(getDrivetrain().getRobotTransform(), velocity,
                getRobotSimulator().getPeriodTime());

        monitor.end();
        monitor.printMillis();
        if (!updatedPath) {
            getRobotSimulator().getGraph().addToSeries("Path", GraphUtil
                    .populateSeries(new XYSeriesWithRenderer("Path"),
                            GraphUtil.parametric(follower.getCurrentPath(), 0.01, .1)));
            updatedPath = true;
        }

        getRobotSimulator().getGraph().changeSeries("Circle", GraphUtil
                .populateSeries(new XYSeriesWithRenderer("Circle"), GraphUtil.circle(follower.getPursuitCircle())));
        Transform closestTransform = follower.getExpectedPathTransform();
        getRobotSimulator().getGraph().changeSeries("Point", GraphUtil.populateSeries(new XYSeriesWithRenderer("Point"), GraphUtil.arrow(closestTransform, .1, .1)));
        getRobotSimulator().getGraph().changeSeries("Point1", GraphUtil.populateSeries(new XYSeriesWithRenderer("Point"), GraphUtil.arrow(follower.getCurrentPath().getTransformFromLength(follower.getTraveledDistance() + 0), .1, .1)));
//        follower.setPreviousTransformOnPath(closestTransform);
        getDrivetrain().setVelocityControl(newVelocity.getLeft(), newVelocity.getRight());
//        getDrivetrain().setPercentOutput(1, 1);
//        graph.addToSeries("Velocity", new XYDataItem(time, newVelocity.getLinear()));
//        graph.addToSeries("Curvature Slowdown", new XYDataItem(time, follower.getCurvatureSlowdownVelocity()));
//        graph.addToSeries("slowdown", new XYDataItem(time, follower.getCurvatureSlowdownVelocity()));
//        graph.addToSeries("Velocity1", new XYDataItem(time, (getRobotSimulator().getRobot().getDrivetrain().getDrivetrainModel().getRightVelocity() + getRobotSimulator().getRobot().getDrivetrain().getDrivetrainModel().getLeftVelocity()) / 2));
//        graph.addToSeries("Position", new XYDataItem(time, follower.getTraveledDistance()));
//        graph.addToSeries("Position Setpoint", new XYDataItem(time, follower.getCurrentPath().getGaussianQuadratureLength()));
    }
}
