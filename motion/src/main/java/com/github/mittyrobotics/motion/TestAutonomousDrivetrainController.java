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

import com.github.mittyrobotics.datatypes.motion.DifferentialDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.DrivetrainSpeeds;
import com.github.mittyrobotics.datatypes.path.Trajectory;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;
import com.github.mittyrobotics.motion.pathfollowing.AutonomousDrivetrainController;
import com.github.mittyrobotics.path.generation.TrajectoryGenerator;
import com.github.mittyrobotics.path.generation.splines.QuinticHermiteSpline;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphUtil;
import com.github.mittyrobotics.visualization.RobotGraph;
import com.github.mittyrobotics.visualization.XYSeriesWithRenderer;
import org.jfree.data.xy.XYDataItem;

import javax.swing.*;

public class TestAutonomousDrivetrainController {
    public static void main(String[] args) {
        double maxAcceleration = 200;
        double maxVelocity = 100;
        double maxAngularAcceleration = 10;
        double maxAngularVelocity = Math.PI;
        double trackWidth = 20;

        AutonomousDrivetrainController controller = new AutonomousDrivetrainController(maxAcceleration, maxVelocity, maxAngularAcceleration,maxAngularVelocity, trackWidth);

        TransformWithVelocityAndCurvature
                splineP1 = new TransformWithVelocityAndCurvature(new Transform(0, 0, Math.PI), 1, 1/-50.0);
        TransformWithVelocityAndCurvature splineP2 = new TransformWithVelocityAndCurvature(new Transform(100, 50, Math.PI), 1, 0);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(splineP1, splineP2);

        Graph graph = new Graph();
        graph.scaleGraphToScale(.15, 50, 25);
        graph.addSeries(GraphUtil.populateSeries(XYSeriesWithRenderer.withLines("Series"), GraphUtil.parametric(spline,
                0.01, 2)));
        Graph trajectoryGraph = new Graph();

        Trajectory trajectory = TrajectoryGenerator.getInstance().generateTrajectory(spline,1000, maxAcceleration, maxVelocity, maxAngularAcceleration, maxAngularVelocity, trackWidth);

        RobotGraph robotGraph = new RobotGraph();

        double dt = 0.002;
        Transform previousTransform = splineP1;
        for(double t = 0; t < 1; t += dt){
            DrivetrainSpeeds speeds = DifferentialDriveKinematics.calculateMaxStateFromPoint(previousTransform, spline.getTransform(t), maxVelocity, maxAngularVelocity, trackWidth);
            previousTransform = spline.getTransform(t);
        }
        for(int i = 0; i < trajectory.getSamples(); i++){
            double time = trajectory.getTime()[i];
            trajectoryGraph.addToSeries("Linear Velocity", new XYDataItem(time, trajectory.getLinearVelocity()[i]));
            trajectoryGraph.addToSeries("Angular Velocity", new XYDataItem(time, trajectory.getAngularVelocity()[i]*10));
        }

        previousTransform = splineP1;
        int step = 0;
        robotGraph.addSeries(GraphUtil.populateSeries(XYSeriesWithRenderer.withLines("Path"), GraphUtil.parametric(spline,
                0.01, 2)));
        double linearVelocity = 0;
        double angularVelocity = 0;
        dt = 0.001;
        for(double t = 0; t < 10; t += dt){
            if(t >= trajectory.getTime()[step] && step < 999) {
                step++;
                linearVelocity = trajectory.getLinearVelocity()[step];
                angularVelocity = trajectory.getAngularVelocity()[step];
            }
            else if(step >= 999){
                linearVelocity = 0;
                angularVelocity = 0;
            }

            Transform transform = previousTransform.rotateBy(new Rotation(angularVelocity * dt));
            transform = transform.add(new Transform(transform.getRotation().cos() * (linearVelocity*dt),transform.getRotation().sin() * (linearVelocity*dt)));

            Transform finalTransform = transform;
            SwingUtilities.invokeLater(() -> robotGraph.graphDifferentialDrive(
                    finalTransform, trackWidth, trackWidth*1.25));

            previousTransform = transform;

            try {
                Thread.sleep((long) (dt*1000.0));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
