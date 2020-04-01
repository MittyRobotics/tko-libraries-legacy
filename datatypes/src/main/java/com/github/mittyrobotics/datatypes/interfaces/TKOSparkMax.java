/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.datatypes.interfaces;

import com.revrobotics.*;

public class TKOSparkMax extends CANSparkMax implements PIDInterface, LimitSwitchInterface {

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
        if (encoder == null) {
            encoder = super.getEncoder();
        }
        return encoder;
    }

    @Override
    public CANEncoder getAlternateEncoder() {
        if (altEncoder == null) {
            altEncoder = super.getAlternateEncoder();
        }
        return altEncoder;
    }

    @Override
    public CANPIDController getPIDController() {
        if (controller == null) {
            controller = super.getPIDController();
        }
        return controller;
    }

    @Override
    public CANAnalog getAnalog(CANAnalog.AnalogMode analogMode) {
        setAnalogMode(analogMode);
        return getAnalog();
    }

    public CANAnalog getAnalog() {
        if (analog == null) {
            analog = super.getAnalog(analogMode);
        }
        return analog;
    }

    public void setEncoderType(EncoderType encoderType) {
        encoder = super.getEncoder(encoderType, 0);
    }

    public void setAnalogMode(CANAnalog.AnalogMode analogMode) {
        this.analogMode = analogMode;
    }

    public void set(ControlType controlType, double value) {
        getPIDController().setReference(value, controlType);
    }

    public void configPIDF(double p, double i, double d, double ff, int slotIdx) {
        getPIDController().setP(p, slotIdx);
        getPIDController().setI(i, slotIdx);
        getPIDController().setD(d, slotIdx);
        getPIDController().setFF(ff, slotIdx);
    }

    public double getPositionRaw() {
        return getEncoder().getPosition();
    }

    public double getVelocityRaw() {
        return getEncoder().getVelocity();
    }

    public double getPosition() {
        return getPositionRaw() / ticksPerInch;
    }

    public double getVelocity() { // inches per second
        return getVelocityRaw() / (60 * ticksPerInch);
    }

    public void setTicksPerInch(double ticksPerInch) {
        this.ticksPerInch = ticksPerInch;
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

}