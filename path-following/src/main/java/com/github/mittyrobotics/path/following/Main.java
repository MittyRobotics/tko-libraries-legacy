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

import com.github.mittyrobotics.datatypes.motion.DifferentialDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.util.PathFollowerProperties;
import com.github.mittyrobotics.simulation.sim.RobotSimManager;
import com.github.mittyrobotics.simulation.util.SimSampleDrivetrain;
import com.github.mittyrobotics.simulation.util.SimSampleRobot;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;
import java.util.Random;

public class Main {
    public static void main(String[] args) {
        //Setup the robot sim
        SimSampleRobot robot = new SimSampleRobot();
        RobotSimManager.getInstance()
                .setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(), 125, 7, 2, 20, 30, 0.02);
        RobotGraph.getInstance().getChart().removeLegend();
        SimSampleDrivetrain.getInstance().setupPIDFValues(0.01, 0, 0, 0.08);

        //Set track width of differential drive kinematics
        DifferentialDriveKinematics.getInstance().setTrackWidth(20);

        //Get random values for robot transform
        Random random = new Random();
        double x = random.nextInt(200) - 200;
        double y = random.nextInt(200) - 100.0;
        double heading = random.nextInt(90) - 45;
        //Set robot transform to random values
        SimSampleDrivetrain.getInstance().setOdometry(x, y, heading);

        boolean reversed = false;

        //Create velocity controller
        PathVelocityController velocityController =
                new PathVelocityController(new VelocityConstraints(200, 50, 150), 10, 0, true);

        //Create path properties
        PathFollowerProperties properties =
                new PathFollowerProperties(velocityController, reversed, false);

        //Create pure pursuit properties
        PathFollowerProperties.PurePursuitProperties purePursuitProperties =
                new PathFollowerProperties.PurePursuitProperties(20, 1.2, 40);

        //Create ramsete properties
        PathFollowerProperties.RamseteProperties ramseteProperties =
                new PathFollowerProperties.RamseteProperties(5.0, .7);

        //Setup the path follower
        PathFollower pathFollower = new PathFollower(properties, ramseteProperties);
        pathFollower.setDrivingGoal(new Transform(100, -24, 0));

        //Add original path to graph
        RobotGraph.getInstance()
                .addPath((GraphManager.getInstance()
                        .graphParametric(pathFollower.getCurrentPath(), .05, 3, .2, "spline", Color.green)));

        //Delay before starting
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Loop
        while (true) {
            //Graph
            SwingUtilities.invokeLater(() -> {
                RobotGraph.getInstance().clearGraph();
                RobotGraph.getInstance().addDataset(GraphManager.getInstance()
                        .graphParametricFast(pathFollower.getCurrentPath(), .07, "spline", Color.cyan));
            });

            DrivetrainVelocities currentVelocities = DrivetrainVelocities
                    .calculateFromWheelVelocities(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity(),
                            SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());

            //Update pure pursuit controller and set velocities
            DrivetrainVelocities drivetrainVelocities =
                    pathFollower.updatePathFollower(SimSampleDrivetrain.getInstance().getRobotTransform(),
                            currentVelocities, RobotSimManager.getInstance().getPeriodTime());

            SimSampleDrivetrain.getInstance()
                    .setVelocities(drivetrainVelocities.getLeftVelocity(), drivetrainVelocities.getRightVelocity());

            try {
                Thread.sleep((long) RobotSimManager.getInstance().getPeriodTime() * 1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
