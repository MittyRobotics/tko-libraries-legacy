package com.github.mittyrobotics.motionprofile;

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

    public double getEndTime(){
        return startTime+time;
    }

    public double getX0() {
        return x0;
    }

    public void setX0(double x0) {
        this.x0 = x0;
        calculate();
    }
}
