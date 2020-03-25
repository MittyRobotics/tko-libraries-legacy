package com.github.mittyrobotics.datatypes.interfaces;

public abstract class TKOSubsystem extends SubsystemBase implements IHardware {
    public TKOSubsystem(String name){
        super();
        setName(name);
    }
}