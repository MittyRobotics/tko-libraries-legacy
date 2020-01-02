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

package com.github.mittyrobotics.path.following.simulation;

import com.github.mittyrobotics.datatypes.motion.DifferentialDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.following.PathFollower;
import com.github.mittyrobotics.path.following.util.PathFollowerProperties;
import com.github.mittyrobotics.simulation.rewrite.sim.SimDrivetrain;
import com.github.mittyrobotics.simulation.rewrite.sim.SimRobot;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;
import java.util.Random;

public class PathFollowerSimRobot extends SimRobot {
    private PathFollower pathFollower;

    public PathFollowerSimRobot(SimDrivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public void robotInit() {
        //Set track width of differential drive kinematics
        DifferentialDriveKinematics.getInstance().setTrackWidth(20);

        //Setup PID values
        getDrivetrain().setupPIDFValues(0.1, 0, 0, 0.09);

        boolean reversed = false;

        //Create velocity controller
        PathVelocityController velocityController =
                new PathVelocityController(new VelocityConstraints(100, 100, 150), 0, 0, false);

        //Create path properties
        PathFollowerProperties properties =
                new PathFollowerProperties(velocityController, reversed, false);

        //Create pure pursuit properties
        PathFollowerProperties.PurePursuitProperties purePursuitProperties =
                new PathFollowerProperties.PurePursuitProperties(20, 1.2, 40);

        //Create ramsete properties
        PathFollowerProperties.RamseteProperties ramseteProperties =
                new PathFollowerProperties.RamseteProperties(5.0, .7);

        PathFollower pathFollower = new PathFollower(properties, ramseteProperties);

        this.pathFollower = pathFollower;

        //Get random values for robot transform
        Random random = new Random();
        double x = random.nextInt(200) - 200;
        double y = random.nextInt(200) - 100.0;
        double heading = random.nextInt(90) - 45;
        //Set robot transform to random values
        getDrivetrain().setOdometry(new Transform(0, 0, 0));

        pathFollower.setDrivingGoal(new Transform(48, 48, 0));
    }

    @Override
    public void robotPeriodic() {
        DrivetrainVelocities currentVelocities = DrivetrainVelocities
                .calculateFromWheelVelocities(getDrivetrain().getDrivetrainModel().getLeftVelocity(),
                        getDrivetrain().getDrivetrainModel().getRightVelocity());

        //Update pure pursuit controller and set velocities
        DrivetrainVelocities drivetrainVelocities =
                pathFollower.updatePathFollower(getDrivetrain().getRobotTransform(),
                        currentVelocities, getRobotSimulator().getPeriodTime());

        getDrivetrain()
                .setVelocityControl(drivetrainVelocities.getLeftVelocity(), drivetrainVelocities.getRightVelocity());

        updateGraph();
    }

    private void updateGraph() {
        //Graph
        SwingUtilities.invokeLater(() -> {
            getRobotSimulator().getGraph().clearGraph();
            getRobotSimulator().getGraph().addDataset(GraphManager.getInstance()
                    .graphParametricFast(pathFollower.getCurrentPath(), .01, "spline", Color.cyan));
        });
    }
}
