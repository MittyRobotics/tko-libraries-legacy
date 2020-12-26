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
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.controllers.PathVelocityController;
import com.github.mittyrobotics.motion.controllers.PurePursuitController;
import com.github.mittyrobotics.motion.pathfollowing.PathFollowerProperties;
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
        PathVelocityController velocityController = new PathVelocityController(1, 0.1, 50, 0, 0);
        double trackWidth = 20;
        boolean reversed = false;
        getDrivetrain().setupPIDFValues(0, 0, 0, 12.0 / 162.0);
        follower =
                new PurePursuitController(new PathFollowerProperties(velocityController, trackWidth, reversed, false),
                        new PathFollowerProperties.PurePursuitProperties(25, 0, 0));
        follower.setDrivingGoal(new Transform(100, 100));

    }

    @Override
    public void robotPeriodic() {
        time += getRobotSimulator().getPeriodTime();
        DrivetrainState velocity = DrivetrainState
                .fromWheelSpeeds(getDrivetrain().getDrivetrainModel().getLeftVelocity() * Conversions.M_TO_IN,
                        getDrivetrain().getDrivetrainModel().getRightVelocity() * Conversions.M_TO_IN,
                        follower.getProperties().trackWidth);
        DrivetrainState newVelocity = follower.updatePathFollower(getDrivetrain().getRobotTransform().mToIn(), velocity,
                getRobotSimulator().getPeriodTime());
        if (!updatedPath) {
            getRobotSimulator().getGraph().addToSeries("Path", GraphUtil
                    .populateSeries(new XYSeriesWithRenderer("Path"),
                            GraphUtil.parametric(follower.getCurrentPath(), 0.01, 1)));
            updatedPath = true;
        }

        getRobotSimulator().getGraph().changeSeries("Circle", GraphUtil
                .populateSeries(new XYSeriesWithRenderer("Circle"), GraphUtil.circle(follower.getPursuitCircle())));

//        System.out.println(newVelocity.getLeft() + " " + newVelocity.getRight());

//        getDrivetrain().setPercentOutput(1, 1);
        System.out.println(getDrivetrain().getDrivetrainModel().getLeftVelocity() * Conversions.M_TO_IN + " " +
                getDrivetrain().getDrivetrainModel().getRightVelocity() * Conversions.M_TO_IN);
//        graph.addToSeries("Velocity", new XYDataItem(time, getDrivetrain().getDrivetrainModel().getLeftVelocity()));
        getDrivetrain().setVelocityControl(newVelocity.getLeft(), newVelocity.getRight());
//        System.out.println((newVelocity.getLeft()+ " " + newVelocity.getLeft()));
    }
}
