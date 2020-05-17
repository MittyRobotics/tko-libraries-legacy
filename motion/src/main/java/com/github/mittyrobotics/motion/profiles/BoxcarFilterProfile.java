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

public class BoxcarFilterProfile {
    private final double maxVel;
    private final double distance;
    private final double t1;
    private final double t2;
    private final double dt;
    private double t4;
    private double fl1;
    private double fl2;
    private double n;
    private double step;
    private double previousFilter1;
    private double previousVelocity;
    private double previousPosition;
    private CircularFifoQueue<Double> filter1Queue;

    public BoxcarFilterProfile(double maxVel, double distance, double t1, double t2, double dt) {
        this.maxVel = maxVel;
        this.distance = distance;
        this.t1 = t1*1000;
        this.t2 = t2*1000;
        this.dt = dt*1000;
        this.t4 = (distance / maxVel) * 1000;
        this.fl1 = Math.ceil(this.t1 / this.dt);
        this.fl2 = Math.ceil(this.t2 / this.dt);
        this.n = t4 / this.dt;
        this.step = 1;
        this.previousFilter1 = 0;
        this.previousVelocity = 0;
        this.previousPosition = 0;
        this.filter1Queue = new CircularFifoQueue<Double>((int)fl2);
    }

    public MotionState calculate() {
        double input = step < (n + 2) ? 1 : 0;
        double filter1 = Math.max(0, Math.min(1, (previousFilter1 + (input == 1 ? (1 / fl1) : (-1 / fl1)))));

        double filter2 = 0;

        filter1Queue.add(filter1);
        for(int i = 0; i < filter1Queue.size(); i++){
            filter2 += filter1Queue.get(i);
        }

        double velocity = (filter1+filter2)/(1+fl2)*maxVel;
        double position = (velocity+previousVelocity)/2*dt/1000+previousPosition;
        double acceleration = (velocity-previousVelocity)/(dt/1000);

        step++;
        previousFilter1 = filter1;
        previousVelocity = velocity;
        previousPosition = position;

        return new MotionState(position, velocity, acceleration);
    }
}
