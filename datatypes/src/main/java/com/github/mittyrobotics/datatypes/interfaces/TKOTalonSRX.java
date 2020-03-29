package com.github.mittyrobotics.datatypes.interfaces;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TKOTalonSRX extends WPI_TalonSRX {
    //Use these if you want to have limit switches but they are wired through the roborio
    private TKODigitalInput forwardLimitSwitch, reverseLimitSwitch;

    public TKOTalonSRX(int deviceNumber) {
        super(deviceNumber);
        configFactoryDefault();
    }

    public void configRoborioForwardLimitSwitch(int id, boolean inversion){
        forwardLimitSwitch = new TKODigitalInput(id);
        forwardLimitSwitch.setInverted(inversion);
    }

    public void configRoborioReverseLimitSwitch(int id, boolean inversion){
        reverseLimitSwitch = new TKODigitalInput(id);
        reverseLimitSwitch.setInverted(inversion);
    }

    public void configRoborioForwardLimitSwitch(int id){
        configRoborioForwardLimitSwitch(id, false);
    }

    public void configRoborioReverseLimitSwitch(int id){
        configRoborioReverseLimitSwitch(id, false);
    }

    public boolean getRoborioForwardLimitSwitch(){
        if(forwardLimitSwitch == null){
            return false;
        } else {
            return forwardLimitSwitch.get();
        }
    }

    public boolean getRoborioReverseLimitSwitch(){
        if(reverseLimitSwitch == null){
            return false;
        } else {
            return reverseLimitSwitch.get();
        }
    }

}