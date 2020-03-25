package com.github.mittyrobotics.datatypes.interfaces;

public abstract class TKOMotorSubsystem extends TKOSubsystem {

    public TKOMotorSubsystem(String name) {
        super(name);
    }

    abstract void overrideSetMotor(double percent);

    public void setMotor(double percent){
        overrideSetMotor(percent);
    }

    public void stopMotor(){
        overrideSetMotor(0);
    }

    public void resetEncoder(){

    }

    public double getPosition(){
        return 0;
    }

    public double getVelocity(){
        return 0;
    }

    public boolean getSwitch(){
        return false;
    }

}