package com.github.mittyrobotics.motionprofile;

public class TrapezoidTimeSegment {
    private double tAccel;
    private double tCruise;
    private double tDecel;
    private double maxAcceleration;

    public TrapezoidTimeSegment(double tAccel, double tCruise, double tDecel, double maxAcceleration) {
        this.tAccel = tAccel;
        this.tCruise = tCruise;
        this.tDecel = tDecel;
        this.maxAcceleration = maxAcceleration;
    }

    public double getTAccel() {
        return tAccel;
    }

    public void setTAccel(double tAccel) {
        this.tAccel = tAccel;
    }

    public double getTCruise() {
        return tCruise;
    }

    public void setTCruise(double tCruise) {
        this.tCruise = tCruise;
    }

    public double getTDecel() {
        return tDecel;
    }

    public void setTDecel(double tDecel) {
        this.tDecel = tDecel;
    }

    public double getTTotal() {
        return tAccel + tCruise + tDecel;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }
}
