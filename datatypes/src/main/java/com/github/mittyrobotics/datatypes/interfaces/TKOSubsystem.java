package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TKOSubsystem extends SubsystemBase implements IHardware {
    public TKOSubsystem(String name){
        super();
        setName(name);
    }
}