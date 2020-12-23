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

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.motion.DifferentialDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.DrivetrainSpeeds;
import com.github.mittyrobotics.datatypes.path.Trajectory;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.controllers.RamseteController;
import com.github.mittyrobotics.motion.pathfollowing.AutonomousDrivetrainController;
import com.github.mittyrobotics.path.generation.Path;
import com.github.mittyrobotics.path.generation.PathGenerator;
import com.github.mittyrobotics.path.generation.TrajectoryGenerator;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphUtil;
import com.github.mittyrobotics.visualization.RobotGraph;
import com.github.mittyrobotics.visualization.XYSeriesWithRenderer;
import org.jfree.data.xy.XYDataItem;

import javax.swing.*;

public class TestAutonomousDrivetrainController {
    public static void main(String[] args) {
        double maxAcceleration = 75;
        double maxVelocity = 100;
        double maxAngularAcceleration = 3;
        double maxAngularVelocity = 5;
        double trackWidth = 20;

        AutonomousDrivetrainController controller =
                new AutonomousDrivetrainController(maxAcceleration, maxVelocity, maxAngularAcceleration,
                        maxAngularVelocity, trackWidth);

        TransformWithVelocityAndCurvature
                splineP0 = new TransformWithVelocityAndCurvature(new Transform(0, 0, Math.toRadians(90)), 0, 0);

        TransformWithVelocityAndCurvature
                splineP1 = new TransformWithVelocityAndCurvature(new Transform(0, 50, Math.toRadians(89)), 1, 0);
        TransformWithVelocityAndCurvature splineP2 =
                new TransformWithVelocityAndCurvature(new Transform(100, 50, Math.toRadians(-45)), 0, 0);
        TransformWithVelocityAndCurvature splineP3 =
                new TransformWithVelocityAndCurvature(new Transform(100, 50, Math.toRadians(-90)), 60, 0);
        TransformWithVelocityAndCurvature splineP5 =
                new TransformWithVelocityAndCurvature(new Transform(159, 50, Math.toRadians(90)), 0, 0);

        Path path = new Path(PathGenerator.getInstance()
                .generateQuinticHermiteSplinePath(new TransformWithVelocityAndCurvature[]{splineP0, splineP1, splineP3, splineP5}));

        Graph graph = new Graph();
        graph.scaleGraphToScale(.15, 50, 25);

        Graph trajectoryGraph = new Graph();


        double[] parameterization = TrajectoryGenerator.getInstance().parameterizePath(path, .01, 1);

        int samples = parameterization.length;

        Trajectory trajectory = TrajectoryGenerator.getInstance()
                .generateTrajectory(path, parameterization, maxAcceleration, maxVelocity, maxAngularAcceleration,
                        maxAngularVelocity, trackWidth);

        RobotGraph robotGraph = new RobotGraph();


        double dt = 1.0 / samples;
        Transform previousTransform = path.getTransform(0);
        for (double t = 0; t < 1; t += dt) {
            DrivetrainSpeeds speeds = DifferentialDriveKinematics
                    .calculateMaxStateFromPoint(previousTransform, path.getTransform(t), maxVelocity,
                            maxAngularVelocity, trackWidth);
            previousTransform = path.getTransform(t);
        }

        trajectoryGraph.addSeries(new XYSeriesWithRenderer("Linear Velocity", null, true, false, null));
        trajectoryGraph.addSeries(new XYSeriesWithRenderer("Angular Velocity", null, true, false, null));
        for (int i = 0; i < trajectory.getSamples(); i++) {
            double time = trajectory.getTime()[i];
            trajectoryGraph.addToSeries("Linear Velocity", new XYDataItem(time, trajectory.getLinearVelocity()[i]));
            trajectoryGraph
                    .addToSeries("Angular Velocity", new XYDataItem(time, trajectory.getAngularVelocity()[i] * 10));
//            trajectoryGraph.addToSeries("Curvature", new XYDataItem(time, trajectory.getCurvature()[i]*1000));
        }

        previousTransform = path.getTransform(0);;
        int step = 0;
        robotGraph
                .addSeries(GraphUtil.populateSeries(XYSeriesWithRenderer.withLines("Path"), GraphUtil.parametric(path,
                        parameterization, .1)));

        graph.addSeries(GraphUtil.populateSeries(XYSeriesWithRenderer.withLines("Series"), GraphUtil.parametric(path,
                parameterization, .1)));
        double linearVelocity = 0;
        double angularVelocity = 0;
        dt = 0.001;
        Transform desiredTransform = splineP1;
        for (double t = 0; t < 100; t += dt) {
            if (t >= trajectory.getTime()[step] && step < trajectory.getSamples() - 1) {
                step++;
                linearVelocity = trajectory.getLinearVelocity()[step];
                angularVelocity = trajectory.getAngularVelocity()[step];
                desiredTransform = path.getTransform(parameterization[step]);
            } else if (step >=  trajectory.getSamples() - 1) {
                linearVelocity = 0;
                angularVelocity = 0;
                desiredTransform = path.getTransform(1);
            }


            DrivetrainSpeeds ramseteSpeeds = RamseteController
                    .calculate(previousTransform.inToM(), desiredTransform.inToM(),
                            linearVelocity * Conversions.IN_TO_M, angularVelocity * Conversions.IN_TO_M, 2.0, 0.2,
                            trackWidth * Conversions.IN_TO_M, false);


            Transform finalDesiredTransform = desiredTransform;
            Transform finalPreviousTransform = previousTransform;
            SwingUtilities.invokeLater(()->{
                robotGraph.changeSeries("Robot Pt Series", GraphUtil.populateSeries(new XYSeriesWithRenderer("Robot Pt Series"), GraphUtil.arrow(
                        finalPreviousTransform, 5, 2)));
            });



            Transform transform =
                    previousTransform.rotateBy(new Rotation(ramseteSpeeds.getAngular() * Conversions.M_TO_IN * dt));
            transform = transform.add(new Transform(
                    transform.getRotation().cos() * (ramseteSpeeds.getLinear() * Conversions.M_TO_IN * dt),
                    transform.getRotation().sin() * (ramseteSpeeds.getLinear() * Conversions.M_TO_IN * dt)));

            robotGraph.scaleGraphToScale(.2, transform.getPosition().getX(), transform.getPosition().getY());

            Transform finalTransform = transform;
            SwingUtilities.invokeLater(() -> robotGraph.graphDifferentialDrive(
                    finalTransform, trackWidth, trackWidth * 1.25));

            previousTransform = transform;

            try {
                Thread.sleep((long) (dt * 1000.0));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
