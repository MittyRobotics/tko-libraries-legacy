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
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    public Trajectory generateTrajectory(Parametric parametric, int samples, double maxAcceleration, double maxVelocity,
                                         double maxAngularAcceleration, double maxAngularVelocity, double trackWidth) {
        double[] linearVelocities = new double[samples];
        double[] angularVelocities = new double[samples];
        double[] curvatures = new double[samples];
        double[] times = new double[samples];

        Iteration lastIteration = new Iteration(0, 0, 0, 0, parametric.getTransform(1));

        //Backward pass
        for (int i = samples - 1; i >= 0; i--) {
            Iteration iteration =
                    doIteration(i, lastIteration, maxVelocity, maxAngularVelocity, maxAcceleration, maxVelocity,
                            maxAngularAcceleration,
                            maxAngularVelocity, trackWidth, parametric, samples);
            linearVelocities[i] = iteration.getLinear();
            angularVelocities[i] = iteration.getAngular();
            curvatures[i] = iteration.getCurvature();
            times[i] = iteration.getTime();
            lastIteration = iteration;
        }

        lastIteration = new Iteration(0, 0, 0, 0, parametric.getTransform(0));
        //Forward Pass
        for (int i = 0; i < samples; i++) {
            Iteration iteration =
                    doIteration(i, lastIteration, linearVelocities[i], angularVelocities[i], maxAcceleration,
                            maxVelocity, maxAngularAcceleration,
                            maxAngularVelocity, trackWidth, parametric, samples);
            linearVelocities[i] = iteration.getLinear();
            angularVelocities[i] = iteration.getAngular();
            curvatures[i] = iteration.getCurvature();
            times[i] = iteration.getTime();
            lastIteration = iteration;
        }

        return new Trajectory(linearVelocities, angularVelocities, curvatures, times);
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

        //Linear velocity cap

        linear = Math.sqrt(lastIteration.getLinear() * lastIteration.getLinear() + 2 * maxAcceleration * length);
        linear = Math.min(linear, maxSpeeds.getLinear());
        linear = Math.min(linear, currentLinearVelocity);

        //Angular velocity cap

        double maxAngularFromLinear =
                DrivetrainSpeeds.fromLinearAndRadius(linear, 1 / curvature, trackWidth).getAngular();

        double angularSqrt =
                lastIteration.getAngular() * lastIteration.getAngular() * Math.signum(lastIteration.getAngular()) +
                        2 * maxAngularAcceleration * angle * Math.signum(curvature);
        angular = Math.sqrt(Math.abs(angularSqrt)) * Math.signum(angularSqrt);
        angular = Math.min(Math.abs(angular), Math.abs(maxAngularFromLinear)) * Math.signum(angular);
        angular = Math.min(Math.abs(angular), Math.abs(currentAngularVelocity)) * Math.signum(angular);

        //Linear velocity adjustment

        if (Math.abs(1 / curvature) < 10000) {
            double adjustLinear = DrivetrainSpeeds.fromAngularAndRadius(angular, 1 / curvature, trackWidth).getLinear();
            linear = adjustLinear;
        }

        //Time calculation

        double linearTime = length / ((linear + lastIteration.getLinear()) / 2);
        double angularTime = angle / ((angular + lastIteration.getAngular()) / 2);


        time = Math.max(linearTime, angularTime);
        time = (Double.isNaN(time) || Double.isInfinite(time)) ? 0 : time;
        time = time + lastIteration.getTime();

        return new Iteration(linear, angular, time, curvature, transform);
    }

    private class Iteration {
        private final double linear;
        private final double angular;
        private final double time;
        private final double curvature;
        private final Transform transform;

        public Iteration(double linear, double angular, double time, double curvature, Transform transform) {
            this.linear = linear;
            this.angular = angular;
            this.time = time;
            this.curvature = curvature;
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

        public double getCurvature() {
            return curvature;
        }

        public Transform getTransform() {
            return transform;
        }
    }

}