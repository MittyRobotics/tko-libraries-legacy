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

package com.github.mittyrobotics.motion.profiles;

import com.github.mittyrobotics.datatypes.motion.MotionState;

public class DynamicTrapezoidalMotionProfile {
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxVelocity;
    private OverrideMethod overrideMethod;

    public DynamicTrapezoidalMotionProfile(double maxAcceleration, double maxDeceleration,
                                           double maxVelocity, OverrideMethod overrideMethod) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.overrideMethod = overrideMethod;
    }

    public MotionState calculateNextState(MotionState currentState, MotionState setpoint, double deltaTime) {
        double position = currentState.getPosition();
        double velocity = currentState.getVelocity();
        double acceleration = currentState.getAcceleration();

        double absPositionError = setpoint.getPosition() - currentState.getPosition();

        double maxDistanceVelocity = Math.sqrt(2 * maxDeceleration * absPositionError);
        double desiredVelocity = Math.min(maxVelocity, maxDistanceVelocity);

        velocity += maxAcceleration * deltaTime;
        velocity = Math.min(velocity, desiredVelocity);

        position += velocity * deltaTime;
        return new MotionState(position, velocity, acceleration);
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

    public OverrideMethod getOverrideMethod() {
        return overrideMethod;
    }
}
