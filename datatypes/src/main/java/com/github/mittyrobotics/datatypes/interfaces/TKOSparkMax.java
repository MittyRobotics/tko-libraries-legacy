package com.github.mittyrobotics.datatypes.interfaces;

import com.revrobotics.*;

public class TKOSparkMax extends CANSparkMax {

    private CANPIDController controller;
    private CANEncoder encoder, altEncoder;
    private CANAnalog analog;
    private CANAnalog.AnalogMode analogMode;
    private double ticksPerInch;
    private TKODigitalInput forwardLimitSwitch, reverseLimitSwitch;

    public TKOSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        super.restoreFactoryDefaults();
        setTicksPerInch(1);
    }

    @Override
    public CANEncoder getEncoder() {
        if(encoder == null){
            encoder = super.getEncoder();
        }
        return encoder;
    }

    @Override
    public CANEncoder getAlternateEncoder() {
        if(altEncoder == null){
            altEncoder = super.getAlternateEncoder();
        }
        return altEncoder;
    }

    @Override
    public CANPIDController getPIDController() {
        if(controller == null){
            controller = super.getPIDController();
        }
        return controller;
    }

    @Override
    public CANAnalog getAnalog(CANAnalog.AnalogMode analogMode) {
        setAnalogMode(analogMode);
        return getAnalog();
    }

    public CANAnalog getAnalog(){
        if(analog == null){
            analog = super.getAnalog(analogMode);
        }
        return analog;
    }

    public void setEncoderType(EncoderType encoderType){
        encoder = super.getEncoder(encoderType, 0);
    }

    public void setAnalogMode(CANAnalog.AnalogMode analogMode){
        this.analogMode = analogMode;
    }

    public void set(ControlType controlType, double value){
        getPIDController().setReference(value, controlType);
    }

    public void setPIDF(double p, double i, double d, double ff, int slotIdx){
        getPIDController().setP(p, slotIdx);
        getPIDController().setI(i, slotIdx);
        getPIDController().setD(d, slotIdx);
        getPIDController().setFF(ff, slotIdx);
    }

    public void setPIDF(double p, double i, double d, double ff){
        setPIDF(p, i, d, ff, 0);
    }

    public void setPID(double p, double i, double d, int slotIdx){
        setPIDF(p, i, d, 0, slotIdx);
    }

    public void setPID(double p, double i, double d){
        setPID(p, i, d, 0);
    }

    public double getPositionRaw(){
        return getEncoder().getPosition();
    }

    public double getVelocityRaw(){
        return getEncoder().getVelocity();
    }

    public double getPosition(){
        return getPositionRaw() / ticksPerInch;
    }

    public double getVelocity(){ // inches per second
        return getVelocityRaw() / (60 * ticksPerInch);
    }

    public void setTicksPerInch(double ticksPerInch){
        this.ticksPerInch = ticksPerInch;
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