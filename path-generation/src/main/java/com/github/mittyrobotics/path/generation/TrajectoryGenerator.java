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
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

import java.util.ArrayList;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    public double[] parameterizePath(Parametric parametric, double maxAngleDelta, double maxDistanceDelta) {
        ArrayList<Double> tValues = new ArrayList<>();
        double t = 0;
        tValues.add(t);
        while (t < 0.999) {
            t = doParameterizeIteration(parametric, t, maxAngleDelta, maxDistanceDelta);
            tValues.add(t);
            System.out.println(t);
        }
        return tValues.stream().mapToDouble(Double::doubleValue).toArray();
    }

    private double doParameterizeIteration(Parametric parametric, double previousT, double maxAngleDelta,
                                           double maxDistanceDelta) {
        Transform previousTransform = parametric.getTransform(previousT);
        double t = previousT + 0.001;
        double distance = Double.POSITIVE_INFINITY;
        double angle = Double.POSITIVE_INFINITY;
        while (distance >= maxDistanceDelta || angle >= maxAngleDelta) {
            Transform transform = parametric.getTransform(t);
            distance = transform.getPosition().distance(previousTransform.getPosition());
            angle = transform.getRotation().subtract(previousTransform.getRotation()).abs().getRadians();
            t -= 0.0000001;
        }
        return t;
    }

    public Trajectory generateTrajectory(Parametric parametric, double[] parameterization, double maxAcceleration,
                                         double maxVelocity,
                                         double maxAngularAcceleration, double maxAngularVelocity, double trackWidth) {
        int arrayLength = parameterization.length;
        double[] linearVelocities = new double[arrayLength];
        double[] angularVelocities = new double[arrayLength];
        double[] curvatures = new double[arrayLength];
        double[] times = new double[arrayLength];
        double lastLinear;
        double lastAngular;

        //Initial velocities cap from curvature
        for (int i = 0; i < arrayLength; i++) {
            double t = parameterization[i];
            double curvature = parametric.getCurvature(t);
            curvatures[i] = curvature;

            DrivetrainSpeeds maxSpeeds = DifferentialDriveKinematics
                    .calculateMaxStateFromCurvature(Math.abs(curvature), maxVelocity, maxAngularVelocity, trackWidth);

            linearVelocities[i] = maxSpeeds.getLinear();
            angularVelocities[i] = maxSpeeds.getAngular();
        }

        lastLinear = 0;
        //Linear forward acceleration cap
        for (int i = 0; i < arrayLength; i++) {
            linearVelocities[i] =
                    linearPass(i, parametric, parameterization, linearVelocities, maxAcceleration, lastLinear);
            lastLinear = linearVelocities[i];
        }

        lastLinear = 0;
        //Linear backward acceleration cap
        for (int i = arrayLength - 1; i >= 0; i--) {
            linearVelocities[i] =
                    linearPass(i, parametric, parameterization, linearVelocities, maxAcceleration, lastLinear);
            lastLinear = linearVelocities[i];
        }

        lastAngular = 0;
        //Angular forward acceleration cap
        for (int i = 0; i < arrayLength; i++) {
            angularVelocities[i] =
                    angularPass(i, parametric, parameterization, angularVelocities, linearVelocities, curvatures,
                            maxAngularAcceleration, lastAngular, trackWidth);
            lastAngular = angularVelocities[i];
        }

        lastAngular = 0;
        //Angular backward acceleration cap
        for (int i = arrayLength - 1; i >= 0; i--) {
            angularVelocities[i] =
                    angularPass(i, parametric, parameterization, angularVelocities, linearVelocities, curvatures,
                            maxAngularAcceleration, lastAngular, trackWidth);
            lastAngular = angularVelocities[i];
        }

        //Linear cap from angular
        for (int i = 0; i < arrayLength; i++) {
            double angular = angularVelocities[i];
            double curvature = curvatures[i];
            linearVelocities[i] = DrivetrainSpeeds.fromAngularAndRadius(angular, 1 / curvature, trackWidth).getLinear();
        }

        //Compute time values
        for (int i = 0; i < arrayLength; i++) {
            double t = parameterization[i];

            Transform transform = parametric.getTransform(t);
            Transform previousTransform = parametric.getTransform(parameterization[Math.max(i - 1, 0)]);

            double length = transform.getPosition().distance(previousTransform.getPosition());
            double angle = transform.getRotation().subtract(previousTransform.getRotation()).abs().getRadians();

            double linear = linearVelocities[i];
            double angular = angularVelocities[i];
            lastLinear = linearVelocities[Math.max(i - 1, 0)];
            lastAngular = angularVelocities[Math.max(i - 1, 0)];
            double lastTime = times[Math.max(i - 1, 0)];

            double linearTime = length / ((linear + lastLinear) / 2);
            double angularTime = Math.abs(angle) / ((Math.abs(angular) + Math.abs(lastAngular)) / 2);

            double time = Math.max(linearTime, angularTime);
            time = (Double.isNaN(time) || Double.isInfinite(time)) ? 0 : time;
            time = time + lastTime;
            times[i] = time;
        }

        return new Trajectory(linearVelocities, angularVelocities, curvatures, times);
    }

    private double linearPass(int i, Parametric parametric, double[] parameterization, double[] linearVelocities,
                              double maxAcceleration, double lastLinear) {
        double t = parameterization[i];

        Position position = parametric.getPosition(t);
        Position previousPosition = parametric.getPosition(parameterization[Math.max(i - 1, 0)]);

        double length = position.distance(previousPosition);
        double currentLinear = linearVelocities[Math.max(i, 0)];

        double linear = Math.sqrt(lastLinear * lastLinear + 2 * maxAcceleration * length);
        linear = Math.min(linear, currentLinear);

        System.out.println(linear);

        return linear;
    }

    private double angularPass(int i, Parametric parametric, double[] parameterization, double[] angularVelocities,
                               double[] linearVelocities, double[] curvatures, double maxAngularAcceleration,
                               double lastAngular, double trackWidth) {
        double t = parameterization[i];

        double currentAngular = angularVelocities[Math.max(i, 0)];
        double curvature = curvatures[i];

        Rotation rotation = parametric.getRotation(t);
        Rotation previousRotation = parametric.getRotation(parameterization[Math.max(i - 1, 0)]);

        double angle = rotation.subtract(previousRotation).abs().getRadians();

        double linear = linearVelocities[i];

        double maxAngularFromLinear =
                DrivetrainSpeeds.fromLinearAndRadius(linear, 1 / curvature, trackWidth).getAngular();

        double angularSqrt =
                lastAngular * lastAngular * Math.signum(lastAngular) +
                        2 * maxAngularAcceleration * angle * Math.signum(curvature);
        double angular = Math.sqrt(Math.abs(angularSqrt)) * Math.signum(angularSqrt);
        angular = Math.min(Math.abs(angular), Math.abs(currentAngular)) * Math.signum(angular);
        angular = Math.min(Math.abs(angular), Math.abs(maxAngularFromLinear)) * Math.signum(angular);

        return angular;
    }
}