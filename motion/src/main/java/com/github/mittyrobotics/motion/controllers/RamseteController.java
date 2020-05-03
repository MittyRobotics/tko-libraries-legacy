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

package com.github.mittyrobotics.motion.controllers;

import com.github.mittyrobotics.datatypes.motion.DrivetrainSpeeds;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class RamseteController {
    public static double DEFAULT_AGGRESSIVE_GAIN = 2.0;
    public static double DEFAULT_DAMPING_GAIN = 0.2;

    /**
     * Calculates the {@link DrivetrainSpeeds} using on the RAMSETE path following algorithm.
     * <p>
     * Make sure all inputs are in SI units (m, m/s).
     *
     * @param robotTransform
     * @param desiredTransform
     * @param robotVelocity
     * @param drivingCurvature
     * @param aggressiveGain
     * @param dampingGain
     * @param trackWidth
     * @param reversed
     * @return
     */
    public static DrivetrainSpeeds calculate(Transform robotTransform, Transform desiredTransform, double robotVelocity,
                                             double drivingCurvature, double aggressiveGain, double dampingGain,
                                             double trackWidth, boolean reversed) {
        //Get the transform error in meters.
        Transform error = desiredTransform.relativeTo(robotTransform);

        //Calculate linear velocity in meters per second given robot velocity in inches per second
        double linearVelocity = robotVelocity;
        //Calculate the angular velocity in radians per second given the turning radius and the robot velocity.
        double angularVelocity = linearVelocity / (1 / drivingCurvature);

        double eX = error.getPosition().getX();
        double eY = error.getPosition().getY();
        double eTheta = error.getRotation().getRadians();

        //Calculate the Ramsete k value
        double k = 2.0 * dampingGain *
                Math.sqrt(Math.pow(angularVelocity, 2) + aggressiveGain * Math.pow(linearVelocity, 2));

        //Calculate the adjusted linear velocity from the Ramsete algorithm
        double adjustedLinearVelocity = linearVelocity * error.getRotation().cos() + k * eX;

        //Convert linear velocity back into inches per second
        adjustedLinearVelocity = adjustedLinearVelocity;

        //Calculate the adjusted angular velocity from the Ramsete algorithm (stays in radians per second)
        double adjustedAngularVelocity =
                angularVelocity + k * eTheta + aggressiveGain * linearVelocity * error.getRotation().sinc() * eY;

        //Calculate drivetrain state from linear and angular velocity
        DrivetrainSpeeds state = DrivetrainSpeeds
                .fromLinearAndAngular(adjustedLinearVelocity, adjustedAngularVelocity * (reversed ? -1 : 1),
                        trackWidth);

        //Reversed controller
        if (reversed) {
            state = state.reverse();
        }

        return state;
    }
}
