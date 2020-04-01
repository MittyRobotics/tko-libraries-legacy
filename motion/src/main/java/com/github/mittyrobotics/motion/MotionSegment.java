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

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.geometry.Line;

public class MotionSegment {
    private Line accelerationLine;
    private double startTime;
    private double time;
    private double v0;
    private double x0;
    private double position;
    private double velocity;

    public MotionSegment(Line accelerationLine, double startTime, double time, double v0, double x0) {
        this.accelerationLine = accelerationLine;
        this.startTime = startTime;
        this.time = time;
        this.v0 = v0;
        this.x0 = x0;
        calculate();
    }

    public void calculate() {
        this.position = getPositionFromTime(time);
        this.velocity = getVelocityFromTime(time);
    }

    public double getPositionFromTime(double t) {
        double m = accelerationLine.getSlope();
        double b = accelerationLine.getYIntercept();
        return m * (t * t * t) / 6 + b * (t * t) / 2 + v0 * t + x0;
    }

    public double getVelocityFromTime(double t) {
        double m = accelerationLine.getSlope();
        double b = accelerationLine.getYIntercept();
        return (m * Math.pow(t, 2) / 2) + b * t + v0;
    }

    public double getAccelerationFromTime(double t) {
        double m = accelerationLine.getSlope();
        double b = accelerationLine.getYIntercept();
        return m * t + b;
    }

    public Line getAccelerationLine() {
        return accelerationLine;
    }

    public void setAccelerationLine(Line accelerationLine) {
        this.accelerationLine = accelerationLine;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getV0() {
        return v0;
    }

    public void setV0(double v0) {
        this.v0 = v0;
        calculate();
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getStartTime() {
        return startTime;
    }

    public void setStartTime(double startTime) {
        this.startTime = startTime;
    }

    public double getEndTime() {
        return startTime + time;
    }

    public double getX0() {
        return x0;
    }

    public void setX0(double x0) {
        this.x0 = x0;
        calculate();
    }
}
