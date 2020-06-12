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

package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.motion.DifferentialDriveKinematics;
import com.github.mittyrobotics.datatypes.motion.DrivetrainSpeeds;
import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.path.Trajectory;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class TrajectoryGenerator {
    private static TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    public Trajectory generateTrajectory(Parametric parametric, int samples, double maxAcceleration, double maxVelocity,
                                         double maxAngularAcceleration, double maxAngularVelocity, double trackWidth) {
        double[] linearVelocities = new double[samples];
        double[] angularVelocities = new double[samples];
        double[] times = new double[samples];

        Iteration lastIteration = new Iteration(0, 0, 0, parametric.getTransform(1));

        //Backward pass
        for (int i = samples - 1; i >= 0; i--) {
            Iteration iteration =
                    doIteration(i, lastIteration, maxVelocity, maxAngularVelocity, maxAcceleration, maxVelocity,
                            maxAngularAcceleration,
                            maxAngularVelocity, trackWidth, parametric, samples);
            linearVelocities[i] = iteration.getLinear();
            angularVelocities[i] = iteration.getAngular();
            times[i] = iteration.getTime();
            lastIteration = iteration;
        }

        lastIteration = new Iteration(0, 0, 0, parametric.getTransform(0));
        //Forward Pass
        for (int i = 0; i < samples; i++) {
            Iteration iteration =
                    doIteration(i, lastIteration, linearVelocities[i], angularVelocities[i], maxAcceleration,
                            maxVelocity, maxAngularAcceleration,
                            maxAngularVelocity, trackWidth, parametric, samples);
            linearVelocities[i] = iteration.getLinear();
            angularVelocities[i] = iteration.getAngular();
            times[i] = iteration.getTime();
            lastIteration = iteration;
        }

        return new Trajectory(linearVelocities, angularVelocities, times);
    }

    private Iteration doIteration(int i, Iteration lastIteration, double currentLinearVelocity,
                                  double currentAngularVelocity, double maxAcceleration, double maxVelocity,
                                  double maxAngularAcceleration, double maxAngularVelocity, double trackWidth,
                                  Parametric parametric, int samples) {
        double t = ((double) i / (double) samples);
        Transform transform = parametric.getTransform(t);
        double curvature = parametric.getCurvature(t);

        double linear;
        double angular;
        double time;

        double length = transform.getPosition().distance(lastIteration.getTransform().getPosition());
        double angle = transform.getRotation().subtract(lastIteration.getTransform().getRotation()).abs().getRadians();

        DrivetrainSpeeds maxSpeeds = DifferentialDriveKinematics
                .calculateMaxStateFromCurvature(Math.abs(curvature), maxVelocity, maxAngularVelocity, trackWidth);
        double adjustedMaxLinearVelocity = maxSpeeds.getLinear();
        double adjustedMaxAngularVelocity = maxSpeeds.getAngular() * Math.signum(curvature);

        //Linear velocity cap

        linear = Math.sqrt(lastIteration.getLinear() * lastIteration.getLinear() + 2 * maxAcceleration * length);
        linear = Math.min(linear, adjustedMaxLinearVelocity);
        linear = Math.min(linear, currentLinearVelocity);

        //Angular velocity final cap

        angular = DrivetrainSpeeds.fromLinearAndRadius(linear, 1 / curvature, trackWidth).getAngular();

        //Time calculation

        double linearTime = length / ((linear + lastIteration.getLinear()) / 2);
        double angularTime = angle / ((angular + lastIteration.getAngular()) / 2);

        time = Math.max(linearTime, angularTime);
        time = (Double.isNaN(time) || Double.isInfinite(time)) ? 0 : time;
        time = time + lastIteration.getTime();

        return new Iteration(linear, angular, time, transform);
    }

    private class Iteration {
        private final double linear;
        private final double angular;
        private final double time;
        private final Transform transform;

        public Iteration(double linear, double angular, double time, Transform transform) {
            this.linear = linear;
            this.angular = angular;
            this.time = time;
            this.transform = transform;
        }

        public double getLinear() {
            return linear;
        }

        public double getAngular() {
            return angular;
        }

        public double getTime() {
            return time;
        }

        public Transform getTransform() {
            return transform;
        }
    }

}


//        double[] curvatures = new double[samples];
//
//        Transform previousTransform = parametric.getTransform(1);
//        double lastLinearVelocity = 0;
//        double lastAngularVelocity = 0;
//        double lastLength = 0;
//        double lastAngle = 0;
//        double lastTime = 0;
//
//        for (int i = samples-1; i >= 0; i--) {
//            double t = ((double) i / (double) samples);
//            Transform transform = parametric.getTransform(t);
//            double curvature = parametric.getCurvature(t);
//
//            double length = transform.getPosition().distance(previousTransform.getPosition());
//            double angle = transform.getRotation().subtract(previousTransform.getRotation()).abs().getRadians();
//
//            DrivetrainSpeeds maxSpeeds = DifferentialDriveKinematics.calculateMaxStateFromCurvature(curvature, maxVelocity, maxAngularVelocity, trackWidth);
//
//            double adjustedMaxLinearVelocity = maxSpeeds.getLinear();
//            double adjustedMaxAngularVelocity = maxSpeeds.getAngular();
//
//            double linearVelocity = Math.sqrt(lastLinearVelocity*lastLinearVelocity + 2*maxAcceleration*length);
//            double angularVelocity = Math.sqrt(lastAngularVelocity*lastAngularVelocity + 2*maxAngularAcceleration*angle);
//
//            linearVelocity = Math.min(linearVelocity, adjustedMaxLinearVelocity);
//            angularVelocity = adjustedMaxAngularVelocity > 0? Math.min(angularVelocity, adjustedMaxAngularVelocity) : Math.min(Math.abs(adjustedMaxAngularVelocity), angularVelocity);
//
//            linearVelocities[i] = linearVelocity;
//            angularVelocities[i] = angularVelocity;
//
//            double linearTime = length/((linearVelocity+lastLinearVelocity)/2);
//            double angularTime = angle/((angularVelocity+lastAngularVelocity)/2);
//
//            double time = Math.max(linearTime, angularTime);
//            if(Double.isNaN(time)){
//                time = 0;
//            }
//
//            time = time + lastTime;
//
//            length = length + lastLength;
//            angle = angle + lastAngle;
//
//            lastLinearVelocity = linearVelocity;
//            lastAngularVelocity = angularVelocity;
//            lastLength = length;
//            lastAngle = angle;
//            previousTransform = transform;
//            lastTime = time;
//        }
//
//        previousTransform = parametric.getTransform(0);
//        lastLinearVelocity = 0;
//        lastAngularVelocity = 0;
//        lastLength = 0;
//        lastAngle = 0;
//        lastTime = 0;
//        for (int i = 0; i < samples; i++) {
//            double t = ((double) i / (double) samples);
//            Transform transform = parametric.getTransform(t);
//            double curvature = parametric.getCurvature(t);
//
//            curvatures[i] = curvature;
//
//            double length = transform.getPosition().distance(previousTransform.getPosition());
//            double angle = transform.getRotation().subtract(previousTransform.getRotation()).abs().getRadians();
//
//            DrivetrainSpeeds maxSpeeds = DifferentialDriveKinematics.calculateMaxStateFromCurvature(curvature, maxVelocity, maxAngularVelocity, trackWidth);
//
//            double adjustedMaxLinearVelocity = linearVelocities[i];
//            double adjustedMaxAngularVelocity = angularVelocities[i];
//
//            double linearVelocity = Math.sqrt(lastLinearVelocity*lastLinearVelocity + 2*maxAcceleration*length);
//            double angularVelocity = Math.sqrt(lastAngularVelocity*lastAngularVelocity + 2*maxAngularAcceleration*angle);
//
//            linearVelocity = Math.min(linearVelocity, adjustedMaxLinearVelocity);
//            angularVelocity = Math.min(angularVelocity, Math.abs(adjustedMaxAngularVelocity));
//
//            linearVelocities[i] = linearVelocity;
//            angularVelocities[i] = angularVelocity;
//
//            double linearTime = length/((linearVelocity+lastLinearVelocity)/2);
//            double angularTime = angle/((angularVelocity+lastAngularVelocity)/2);
//
//            double time = Math.max(linearTime, angularTime);
//            if(Double.isNaN(time)){
//                time = 0;
//            }
//
//            time = time + lastTime;
//            times[i] = time;
//
//
//            length = length + lastLength;
//            angle = angle + lastAngle;
//
//            lastLinearVelocity = linearVelocity;
//            lastAngularVelocity = angularVelocity;
//            lastLength = length;
//            lastAngle = angle;
//            previousTransform = transform;
//            lastTime = time;
//        }
//
//        lastTime = 0;
//        lastLinearVelocity = 0;
//        lastAngularVelocity = 0;
//        for(int i = 0; i < samples; i++){
//            double angular = angularVelocities[i];
//            double linear = DrivetrainSpeeds.fromAngularAndRadius(angular, 1/curvatures[i], trackWidth).getLinear();
//            linearVelocities[i] = -linear;
//
//            double t = ((double) i / (double) samples);
//            Transform transform = parametric.getTransform(t);
//            double length = transform.getPosition().distance(previousTransform.getPosition());
//            double angle = transform.getRotation().subtract(previousTransform.getRotation()).abs().getRadians();
//
//            linear = Math.sqrt(lastLinearVelocity*lastLinearVelocity + 2*maxAcceleration*length);
//            linear = Math.min(linear, linearVelocities[i]);
//
//            double linearTime = length/((linear+lastLinearVelocity)/2);
//            double angularTime = angle/((angular+lastAngularVelocity)/2);
//
//            linearVelocities[i] = linear;
//
//            angularVelocities[i] = -DrivetrainSpeeds.fromLinearAndRadius(linear, 1/curvatures[i], trackWidth).getAngular();
//            double time = Math.max(linearTime, angularTime);
//            if(Double.isNaN(time) || Double.isInfinite(time)){
//                time = 0;
//            }
//
//
//            time = time + lastTime;
//            System.out.println(time);
//            times[i] = time;
//
//            lastTime = time;
//            lastLinearVelocity = linear;
//            lastAngularVelocity = angular;
//            previousTransform = transform;
//        }

