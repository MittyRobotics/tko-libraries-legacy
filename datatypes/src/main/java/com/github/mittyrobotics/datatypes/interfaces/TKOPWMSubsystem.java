package com.github.mittyrobotics.datatypes.interfaces;

public abstract class TKOPWMSubsystem extends TKOSubsystem {

    public TKOPWMSubsystem(String name) {
        super(name);
    }

    abstract void setPosition(double position);

}