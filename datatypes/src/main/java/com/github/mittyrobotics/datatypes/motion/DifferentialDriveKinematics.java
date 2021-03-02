/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.datatypes.motion;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

/**
 * Contains differential drive kinematics equations for figuring out wheel velocities.
 * <p>
 * http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 */
public class DifferentialDriveKinematics {
    /**
     * Calculates the {@link DrivetrainWheelState} given a linear robot movement and a radius of the circle that it
     * wants to follow using differential drive kinematics.
     *
     * @param linear     the linear robot movement in units per second (how fast the robot moves forward).
     * @param radius     the radius to follow in units.
     * @param trackWidth the width between left and right wheels of the drivetrain.
     * @return the calculated {@link DrivetrainWheelState}.
     */
    public static DrivetrainWheelState calculateFromLinearAndRadius(double linear, double radius,
                                                                    double trackWidth) {
        if (Double.isInfinite(radius)) {
            return new DrivetrainWheelState(linear, linear);
        }

        //Calculate the angular velocity of the robot in radians per second
        double angular = linear / radius;

        //Calculate left and right drivetrain velocities
        double left = angular * (radius - (trackWidth / 2));
        double right = angular * (radius + (trackWidth / 2));

        //Return the calculated drivetrain velocities
        return new DrivetrainWheelState(left, right);
    }

    public static DrivetrainWheelState calculateFromAngularAndRadius(double angular, double radius,
                                                                     double trackWidth) {
        if (Double.isInfinite(radius)) {
            return new DrivetrainWheelState(0, 0);
        }

        //Calculate left and right drivetrain velocities
        double left = angular * (radius - (trackWidth / 2));
        double right = angular * (radius + (trackWidth / 2));

        //Return the calculated drivetrain velocities
        return new DrivetrainWheelState(left, right);
    }

    /**
     * Calculates the {@link DrivetrainWheelState} given a robot linear and angular movement.
     *
     * @param linear     the linear robot movement in units per second (how fast the robot moves forward).
     * @param angular    the angular movement of the robot in radians per second
     * @param trackWidth the width between left and right wheels of the drivetrain.
     * @return the calculated {@link DrivetrainWheelState}.
     */
    public static DrivetrainWheelState calculateFromLinearAndAngular(double linear, double angular,
                                                                     double trackWidth) {
        if (linear == 0 && angular == 0) {
            return new DrivetrainWheelState(0, 0);
        }

        //Calculate the radius given linear velocity and angular velocity
        double radius = linear / angular;

        //Return the calculated drivetrain velocities
        return new DrivetrainWheelState(angular * (radius - (trackWidth / 2)),
                angular * (radius + (trackWidth / 2)));
    }

    public static double getRadiusFromWheelSpeeds(DrivetrainWheelState wheelSpeeds, double trackWidth) {
        double linearVelocity = wheelSpeeds.getAvg();

        return linearVelocity / getAngularVelocityFromWheelSpeeds(wheelSpeeds, trackWidth);
    }

    public static double getAngularVelocityFromWheelSpeeds(DrivetrainWheelState wheelSpeeds, double trackWidth) {
        double rightVelocity = wheelSpeeds.getRight();
        double linearVelocity = wheelSpeeds.getAvg();


        double angularVelocity = (2 * (rightVelocity - linearVelocity)) / trackWidth;

        return angularVelocity;
    }

    public static DrivetrainState calculateMaxStateFromPoint(Transform currentTransform, Transform desiredTransform,
                                                             double maxVelocity, double maxAngularVelocity,
                                                             double trackWidth) {
        Circle circle = new Circle(currentTransform, desiredTransform.getPosition());
        if (Double.isInfinite(circle.getRadius()) || Double.isNaN(circle.getRadius())) {
            return DrivetrainState.fromLinearAndAngular(maxVelocity, 0, trackWidth);
        }
        DrivetrainState theoreticalAngularSpeeds =
                DrivetrainState.fromLinearAndRadius(maxVelocity, circle.getRadius(), trackWidth);
        if (theoreticalAngularSpeeds.getAngular() < maxAngularVelocity) {
            return theoreticalAngularSpeeds;
        } else {
            return DrivetrainState.fromAngularAndRadius(maxAngularVelocity, circle.getRadius(), trackWidth);
        }
    }

    public static DrivetrainState calculateMaxStateFromCurvature(double curvature, double maxVelocity,
                                                                 double maxAngularVelocity, double trackWidth) {
        if (Double.isInfinite(curvature) || Double.isNaN(curvature)) {
            return DrivetrainState.fromLinearAndAngular(maxVelocity, 0, trackWidth);
        }
        DrivetrainState theoreticalAngularSpeeds =
                DrivetrainState.fromLinearAndRadius(maxVelocity, 1 / curvature, trackWidth);
        if (theoreticalAngularSpeeds.getAngular() < maxAngularVelocity) {
            return theoreticalAngularSpeeds;
        } else {
            return DrivetrainState.fromAngularAndRadius(maxAngularVelocity, 1 / curvature, trackWidth);
        }
    }

    public static Transform calculateDeltaTransform(DrivetrainState state, double dt){
        double r = state.getDrivingRadius();
        double drivenDist = state.getLinear()*dt;
        Position deltaPos;
        Rotation deltaTheta;
        if(Double.isInfinite(r) || Double.isNaN(r) || Math.abs(r) > 1e3){
            deltaPos = new Position(drivenDist, 0);
            deltaTheta = new Rotation();
        }
        else{
            double c = r*2*Math.PI;
            double theta = (drivenDist/c) * 2*Math.PI;
            deltaPos = new Position(r*Math.cos(theta), r*Math.sin(theta));
            deltaTheta = new Rotation(theta);
        }

        return new Transform(deltaPos, deltaTheta);
    }
}