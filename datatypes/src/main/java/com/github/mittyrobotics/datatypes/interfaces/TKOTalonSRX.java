package com.github.mittyrobotics.datatypes.interfaces;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TKOTalonSRX extends WPI_TalonSRX implements PIDInterface, LimitSwitchInterface {
    //Use these if you want to have limit switches but they are wired through the roborio
    private TKODigitalInput forwardLimitSwitch, reverseLimitSwitch;

    public TKOTalonSRX(int deviceNumber) {
        super(deviceNumber);
        configFactoryDefault();
    }

    public void configRoborioForwardLimitSwitch(int id, boolean inversion) {
        forwardLimitSwitch = new TKODigitalInput(id);
        forwardLimitSwitch.setInverted(inversion);
    }

    public void configRoborioReverseLimitSwitch(int id, boolean inversion) {
        reverseLimitSwitch = new TKODigitalInput(id);
        reverseLimitSwitch.setInverted(inversion);
    }

    public boolean getRoborioForwardLimitSwitch() {
        if (forwardLimitSwitch == null) {
            return false;
        } else {
            return forwardLimitSwitch.get();
        }
    }

    public boolean getRoborioReverseLimitSwitch() {
        if (reverseLimitSwitch == null) {
            return false;
        } else {
            return reverseLimitSwitch.get();
        }
    }

    @Override
    public void configPIDF(double p, double i, double d, double ff, int slotIdx) {
        super.config_kP(slotIdx, p);
        super.config_kI(slotIdx, i);
        super.config_kD(slotIdx, d);
        super.config_kF(slotIdx, ff);
    }
}