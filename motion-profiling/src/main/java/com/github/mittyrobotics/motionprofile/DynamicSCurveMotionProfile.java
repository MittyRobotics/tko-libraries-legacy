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

package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.MotionState;

public class DynamicSCurveMotionProfile {
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxAccelerationAcceleration;
    private double maxVelocity;
    private OverrideMethod overrideMethod;

    /**
     * Creates a new {@link DynamicSCurveMotionProfile}. The s-curve motion profile is a more complex form of the
     * trapezoidal motion profile, resulting in a trapezoidal acceleration over time shape versus a trapezoidal
     * velocity over time shape, and overall resulting in much smoother motion with the tradeoff of slightly less
     * speed.
     * <p>
     * In s-curve motion profiles, because the acceleration over time graph is trapezoidal, there is an overall reduced
     * amount of jerk in the motion versus trapezoidal motion profiles.
     * <p>
     * This motion profile is dynamic, meaning the setpoint can be changed on the fly without a recalculation of the
     * profile. Also, the only variables the motion profile relies on are the current state of the system and
     * setpoint, represented through {@link MotionState}s, and the change in time from the last calculation of the
     * motion profile. This means that whatever state your system is in, you can instantaneously start following a
     * smooth motion profile to the desired setpoint.
     * <p>
     * The motion profile also supports handling cases where the setpoint is impossible to reach from the system's
     * current state. This is the override method, and the types of override include overshooting the setpoint and
     * coming back within the bounds of the system's velocity and acceleration constraints, violating the system's
     * velocity and acceleration constraints to reach the setpoint on time, or stopping as close to the setpoint as
     * possible within the velocity and acceleration constraints without coming back.
     *
     * @param maxAcceleration             the maximum acceleration of the motion profile. (units/s^2)
     * @param maxDeceleration             the maximum deceleration of the motion profile. (units/s^2)
     * @param maxAccelerationAcceleration the maximum acceleration of the acceleration and deceleration values.
     *                                    (units/s^3)
     * @param maxVelocity                 the maximum velocity of the motion profile. (units/s)
     * @param overrideMethod              the method to override the motion profile if the setpoint is impossible to reach at the
     *                                    system's current state.
     */
    public DynamicSCurveMotionProfile(double maxAcceleration, double maxDeceleration,
                                      double maxAccelerationAcceleration, double maxVelocity,
                                      OverrideMethod overrideMethod) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxAccelerationAcceleration = maxAccelerationAcceleration;
        this.maxVelocity = maxVelocity;
        this.overrideMethod = overrideMethod;
    }

    public MotionState calculateNextState(MotionState currentState, MotionState setpoint, double deltaTime) {
        double position = currentState.getPosition();
        double velocity = currentState.getVelocity();
        double acceleration = currentState.getAcceleration();

        double distanceToDecelerate = computeDistanceToStop();

        boolean inDeceleration =
                Math.abs(currentState.getPosition() - setpoint.getPosition()) < distanceToDecelerate;

        if (inDeceleration) {
            double velError = currentState.getVelocity() - setpoint.getVelocity();

            double maxAccelToEnd = Math.sqrt(2 * maxAccelerationAcceleration * velError);
            double desiredAcceleration = -Math.min(maxDeceleration, maxAccelToEnd);
            acceleration -= maxAccelerationAcceleration * deltaTime;
            acceleration = Math.max(acceleration, desiredAcceleration);
        } else {
            double velError = maxVelocity - currentState.getVelocity();

            double maxAccelerationToEnd = Math.sqrt(2 * maxAccelerationAcceleration * velError);
            if (Double.isNaN(maxAccelerationToEnd)) {
                maxAccelerationToEnd = 0;
            }
            double desiredAcceleration = Math.min(maxAcceleration, maxAccelerationToEnd);

            acceleration += maxAccelerationAcceleration * deltaTime;
            acceleration = Math.min(acceleration, desiredAcceleration);
        }

        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;

        return new MotionState(position, velocity, acceleration);
    }

    private double computeDistanceToStop() {
        double theoreticalMaxDeceleration = Math.sqrt(2 * maxAccelerationAcceleration * maxVelocity / 2);
        theoreticalMaxDeceleration = Math.min(theoreticalMaxDeceleration, maxDeceleration);
        double tAccel = theoreticalMaxDeceleration / maxAccelerationAcceleration;
        double tDecel = theoreticalMaxDeceleration / maxAccelerationAcceleration;
        double vTotal = maxVelocity;
        double vAccel = theoreticalMaxDeceleration * tAccel / 2;
        double vDecel = theoreticalMaxDeceleration * tDecel / 2;
        double vCruise = vTotal - vAccel - vDecel;
        double tCruise = vCruise / theoreticalMaxDeceleration;

        double positionAccel = calculateDisplacementFromAccelerationLine(
                tAccel,
                -theoreticalMaxDeceleration / tAccel,
                0,
                maxVelocity
        );

        double positionCruise = (maxVelocity - vAccel - vDecel) * tCruise / 2 + (vDecel * tCruise);

        double positionDecel = calculateDisplacementFromAccelerationLine(
                tDecel,
                theoreticalMaxDeceleration / tDecel,
                -theoreticalMaxDeceleration,
                maxVelocity - (vAccel + vCruise)
        );

        return positionAccel + positionCruise + positionDecel;
    }

    private double calculateDisplacementFromAccelerationLine(double time, double accelerationSlope,
                                                             double accelerationYIntercept, double initialVelocity) {
        double x = time;
        double m = accelerationSlope;
        double v0 = initialVelocity;
        double b = accelerationYIntercept;
        double x0 = 0;
        return m * (x * x * x) / 6 + b * (x * x) / 2 + v0 * x + x0;
    }


    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAccelerationAcceleration() {
        return maxAccelerationAcceleration;
    }

    public OverrideMethod getOverrideMethod() {
        return overrideMethod;
    }
}
