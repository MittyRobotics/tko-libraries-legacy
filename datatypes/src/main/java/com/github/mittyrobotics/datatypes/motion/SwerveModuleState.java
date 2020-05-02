package com.github.mittyrobotics.datatypes.motion;

public class SwerveModuleState {
    private double wheelState;
    private double steerState;

    public SwerveModuleState() {
        this(0, 0);
    }

    public SwerveModuleState(double wheelState, double steerState) {
        this.wheelState = wheelState;
        this.steerState = steerState;
    }

    public double getWheelState() {
        return wheelState;
    }

    public void setWheelState(double wheelState) {
        this.wheelState = wheelState;
    }

    public double getSteerState() {
        return steerState;
    }

    public void setSteerState(double steerState) {
        this.steerState = steerState;
    }
}
