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
import com.github.mittyrobotics.datatypes.positioning.Position;

import java.util.ArrayList;

public class DynamicSCurveMotionProfile {
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxJerk;
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
     * @param maxAcceleration the maximum acceleration of the motion profile. (units/s^2)
     * @param maxDeceleration the maximum deceleration of the motion profile. (units/s^2)
     * @param maxJerk         the maximum jerk of the motion profile. (units/s^3)
     * @param maxVelocity     the maximum velocity of the motion profile. (units/s)
     * @param overrideMethod  the method to override the motion profile if the setpoint is impossible to reach at the
     *                        system's current state.
     */
    public DynamicSCurveMotionProfile(double maxAcceleration, double maxDeceleration,
                                      double maxJerk, double maxVelocity,
                                      OverrideMethod overrideMethod) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxJerk = maxJerk;
        this.maxVelocity = maxVelocity;
        this.overrideMethod = overrideMethod;
    }

    public MotionState calculateNextState(MotionState currentState, MotionState setpoint, double deltaTime) {
        double position = currentState.getPosition();
        double velocity = currentState.getVelocity();
        double acceleration = currentState.getAcceleration();

        boolean reversed = currentState.getPosition() > setpoint.getPosition();

        double distanceToDecelerate = computeDistanceToStop();

        boolean inDeceleration =
                Math.abs(currentState.getPosition() - setpoint.getPosition()) < distanceToDecelerate;


        if (inDeceleration) {
            double velError = currentState.getVelocity() - setpoint.getVelocity();

            double maxAccelToEnd = Math.sqrt(2 * maxJerk * velError);
            double desiredAcceleration = -Math.min(maxDeceleration, maxAccelToEnd);

            acceleration -= maxJerk * deltaTime;
            acceleration = Math.max(acceleration, desiredAcceleration);
        } else {
            double velError = maxVelocity - currentState.getVelocity();

            double maxAccelerationToEnd = Math.sqrt(2 * maxJerk * velError);
            if (Double.isNaN(maxAccelerationToEnd)) {
                maxAccelerationToEnd = 0;
            }
            double desiredAcceleration = Math.min(maxAcceleration, maxAccelerationToEnd);

            acceleration += maxJerk * deltaTime;
            acceleration = Math.min(acceleration, desiredAcceleration);
        }

        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;

        return new MotionState(position, velocity, acceleration, inDeceleration ? 20 : -20);
    }

    private double computeDistanceToStop() {
        double theoreticalMaxDeceleration = Math.sqrt(2 * maxJerk * maxVelocity / 2);
        theoreticalMaxDeceleration = Math.min(theoreticalMaxDeceleration, maxDeceleration);
        double tAccel = theoreticalMaxDeceleration / maxJerk;
        double tDecel = theoreticalMaxDeceleration / maxJerk;
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

    private double computeTheoreticalMaxVelocity(MotionState currentState, MotionState setpoint, double deltaTime) {

        return 0;
    }

    /**
     * Determines whether the motion profile will override at the current state. Returns the distance that it will
     * override. Will output -1 if it will not override.
     *
     * @param currentState   the current {@link MotionState} of the system.
     * @param setpoint       the setpoint state of the motion profile
     * @param deltaTime      the change in time from the previous call of the motion profile.
     * @param distanceToStop the previously computed minimum distance to end the motion profile
     * @return the distance that the motion profile will override. -1 if it will not override.
     */
    private double isOverride(MotionState currentState, MotionState setpoint, double deltaTime, double distanceToStop) {

        return -1;
    }

    public Position[] epicEquation(double y, double m, double b, double x0, double v0) {
        //Common equations
        double a = -(pow(b / 2, 3) / (27 * pow(m / 6, 3))) + ((b / 2 * v0) / (6 * pow(m / 6, 2))) -
                ((x0 - y) / (2 * (m / 6)));
        double c = pow(a, 2) + (pow((v0 / (3 * (m / 6))) - (pow(b / 2, 2) / (9 * pow(m / 6, 2))), 3));
        double e = sqrt(c);
        double f = (-b + sqrt(pow(b, 2) - 2 * m * v0)) / m;
        double f1 = (-b - sqrt(pow(b, 2) - 2 * m * v0)) / m;
        double g = (((m * pow((f), 3)) / 6) + ((b * pow(f, 2)) / 2) + (v0 * f) + x0 + ((m * (pow(f1, 3))) / 6) +
                ((b * pow(f1, 2)) / 2) + (v0 * f1) + x0) / 2;
        double h = pow(pow(a, 2) - c, 1.0 / 6.0);
        double k = atan(sqrt(-c) / a) / 3;
        double o = 2 * h * cos(((2 * Math.PI) / 3) + k);
        double p = 2 * h * cos(((4 * Math.PI) / 3) + k);
        double q = (a / abs(a));
        double u = ((b / 2) / (3 * (m / 6)));

        //Base equations
        double eqn0 = cbrt(a + e) + cbrt(a - e) - ((b / 2) / (3 * (m / 6)));
        double eqn1a = q * (o - u);
        double eqn1b = q * (2 * h * cos(k) + u);
        double eqn2a = q * (p - u);
        double eqn2b = q * (o + u);
        double eqn3a = q * (2 * h * cos(k) - u);
        double eqn3b = q * (p + u);

        //Conditions
        boolean condition1a = y > g;
        boolean condition1b = y <= g;
        boolean condition2a = y > g;
        boolean condition2b = y <= g;
        boolean condition3a = y > g;
        boolean condition3b = y <= g;

        //Form final equations from conditions. If conditions invalid, set value to infinity and check for that later.
        double eqn1, eqn2, eqn3;
        if (condition1a) {
            eqn1 = eqn1a;
        } else if (condition1b) {
            eqn1 = eqn1b;
        } else {
            eqn1 = Double.POSITIVE_INFINITY;
        }
        if (condition2a) {
            eqn2 = eqn2a;
        } else if (condition2b) {
            eqn2 = eqn2b;
        } else {
            eqn2 = Double.POSITIVE_INFINITY;
        }
        if (condition3a) {
            eqn3 = eqn3a;
        } else if (condition3b) {
            eqn3 = eqn3b;
        } else {
            eqn3 = Double.POSITIVE_INFINITY;
        }

        //Form points from equations
        Position p0 = new Position(eqn0, y);
        Position p1 = new Position(eqn1, y);
        Position p2 = new Position(eqn2, y);
        Position p3 = new Position(eqn3, y);

        //Init points list
        ArrayList<Position> points = new ArrayList<>();

        //Add points to point list if they are valid
        if (Double.isFinite(eqn0) && !Double.isNaN(eqn0)) {
            points.add(p0);
        }
        if (Double.isFinite(eqn1) && !Double.isNaN(eqn1)) {
            points.add(p1);
        }
        if (Double.isFinite(eqn2) && !Double.isNaN(eqn2)) {
            points.add(p2);
        }
        if (Double.isFinite(eqn3) && !Double.isNaN(eqn3)) {
            points.add(p3);
        }

        //Return points list to array
        return points.toArray(new Position[0]);
    }

    double sqrt(double x) {
        return Math.sqrt(x);
    }

    double cbrt(double x) {
        return Math.cbrt(x);
    }

    double pow(double x, double degree) {
        return Math.pow(x, degree);
    }

    double atan(double x) {
        return Math.atan(x);
    }

    double cos(double x) {
        return Math.cos(x);
    }

    double abs(double x) {
        return Math.abs(x);
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

    private double computeTimeFromDisplacement(double m, double b, double v0, double x0) {
        return 0;
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

    public double getMaxJerk() {
        return maxJerk;
    }

    public OverrideMethod getOverrideMethod() {
        return overrideMethod;
    }
}
