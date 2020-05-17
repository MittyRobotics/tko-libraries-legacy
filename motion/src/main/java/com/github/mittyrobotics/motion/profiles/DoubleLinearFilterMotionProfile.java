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

package com.github.mittyrobotics.motion.profiles;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import org.apache.commons.collections4.queue.CircularFifoQueue;

public class DoubleLinearFilterMotionProfile {
    private final double maxVel;
    private final double distance;
    private final double filter1Time;
    private final double filter2Time;
    private final double dt;
    private double totalTime;
    private double filter1Length;
    private double filter2Length;
    private double inputs;
    private double step;
    private double previousFilter1;
    private double previousVelocity;
    private double previousPosition;
    private CircularFifoQueue<Double> filter1Queue;

    /**
     * Constructs a double linear filter motion profile.
     *
     * @param maxVel      maximum allowed velocity.
     * @param distance    setpoint distance.
     * @param filter1Time time for first filter, seconds. In other words, it is the time it takes to get from 0
     *                    velocity to max velocity. Larger values result in lower acceleration.
     * @param filter2Time time for second filter, seconds. In other words, it is the time it takes to get from 0
     *                    acceleration to max acceleration (defined by filter1Time). Larger values result in lower jerk.
     * @param dt          delta time between calculate() calls.
     */
    public DoubleLinearFilterMotionProfile(double maxVel, double distance, double filter1Time, double filter2Time,
                                           double dt) {
        this.maxVel = maxVel;
        this.distance = distance;
        this.filter1Time = filter1Time * 1000;
        this.filter2Time = filter2Time * 1000;
        this.dt = dt * 1000;
        this.totalTime = (distance / maxVel) * 1000;
        this.filter1Length = Math.ceil(this.filter1Time / this.dt);
        this.filter2Length = Math.ceil(this.filter2Time / this.dt);
        this.inputs = totalTime / this.dt;
        this.step = 1;
        this.previousFilter1 = 0;
        this.previousVelocity = 0;
        this.previousPosition = 0;
        this.filter1Queue = new CircularFifoQueue<Double>((int) filter2Length);
    }

    public MotionState calculate() {
        double input = step < (inputs + 2) ? 1 : 0;
        double filter1 = Math.max(0, Math.min(1, (previousFilter1 + (input == 1 ? (1 / filter1Length) : (-1 /
                filter1Length)))));

        filter1Queue.add(filter1);

        double filter2 = 0;
        for (int i = 0; i < filter1Queue.size(); i++) {
            filter2 += filter1Queue.get(i);
        }

        double velocity = filter2 / filter2Length * maxVel;
        double position = (velocity + previousVelocity) / 2 * dt / 1000 + previousPosition;
        double acceleration = (velocity - previousVelocity) / (dt / 1000);

        step++;
        previousFilter1 = filter1;
        previousVelocity = velocity;
        previousPosition = position;

        return new MotionState(position, velocity, acceleration);
    }
}
