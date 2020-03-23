package com.github.mittyrobotics.datatypes.interfaces;

public abstract class TKOSolenoidSubsystem extends TKOSubsystem {

    private boolean isExtended;

    public TKOSolenoidSubsystem(String name, boolean startsExtended) {
        super(name);
        this.isExtended = startsExtended;
    }

    public TKOSolenoidSubsystem(String name){
        this(name, false);
    }

    abstract void setSolenoid(boolean extend);

    public void extendSolenoid(){
        setSolenoid(true);
        isExtended = true;
    }

    public void retractSolenoid(){
        setSolenoid(false);
        isExtended = false;
    }

    public boolean isExtended(){
        return isExtended;
    }
}