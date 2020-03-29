package com.github.mittyrobotics.datatypes.interfaces;

public abstract class TKODualMotorSubsystem extends TKOSubsystem {

    public TKODualMotorSubsystem(String name) {
        super(name);
    }

    abstract void overrideSetMotor(double leftPercent, double rightPercent);

    public void setMotor(double leftPercent, double rightPercent) {
        overrideSetMotor(leftPercent, rightPercent);
    }

    public void stopMotor() {
        overrideSetMotor(0, 0);
    }

    public void resetEncoder() {

    }

    public double getLeftPosition() {
        return 0;
    }

    public double getLeftVelocity() {
        return 0;
    }

    public double getRightPosition() {
        return 0;
    }

    public double getRightVelocity() {
        return 0;
    }

    public double getAveragePosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    public boolean getLeftSwitch() {
        return false;
    }

    public boolean getRightSwitch() {
        return false;
    }
}
