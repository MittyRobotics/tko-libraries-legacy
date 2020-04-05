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

/**
 * Enhanced {@link CANSparkMax} with extra features
 */
public class TKOSparkMax extends CANSparkMax implements PIDFInterface, LimitSwitchInterface, TicksConversionInterface {

    /**
     * The built-in {@link CANPIDController} for this {@link TKOSparkMax}
     */
    private CANPIDController controller;

    /**
     * The built-in {@link CANEncoder} for this {@link TKOSparkMax}
     */
    private CANEncoder encoder;

    /**
     * The built-in {@link CANAnalog} for this {@link TKOSparkMax}
     */
    private CANAnalog analog;

    /**
     * The {@link CANAnalog.AnalogMode} for the {@link CANAnalog}
     */
    private CANAnalog.AnalogMode analogMode;

    /**
     * The conversion factor to convert encoder ticks to other units
     */
    private double ticksPerUnit;

    /**
     * Forward and reverse limit switches for limit swtiches that a wired through the roborio
     * If limit switches are wired through the motor controller, use the limit switch methods in {@link CANSparkMax} instead
     */
    private TKODigitalInput forwardLimitSwitch, reverseLimitSwitch;

    /**
     * Constructs a {@link TKOSparkMax} object
     * @param deviceID the id of the motor controller
     * @param type if the motor is brushed or brushless
     */
    public TKOSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        super.restoreFactoryDefaults();
        setTicksPerUnit(1);
        setAnalogMode(CANAnalog.AnalogMode.kRelative);
    }

    /**
     * Returns {@link CANEncoder} configured to this {@link TKOSparkMax}
     * Configures the encoder to a Hall Sensor with cpr of 42 (default for brushless encoder) if the encoder is not configured
     * @return encoder
     */
    @Override
    public CANEncoder getEncoder() {
        if (encoder == null) {
            configEncoder();
        }
        return encoder;
    }

    /**
     * Returns {@link CANEncoder} configured to this {@link TKOSparkMax}
     * Configures the encoder to a Quadrature encoder with cpr of 0
     * @return encoder
     */
    @Override
    public CANEncoder getAlternateEncoder() {
        if (encoder == null) {
            configEncoder(EncoderType.kQuadrature, 0);
        }
        return encoder;
    }

    /**
     * Returns a {@link CANPIDController} configured to this {@link TKOSparkMax}
     * Instantiates controller if null
     * @return controller
     */
    @Override
    public CANPIDController getPIDController() {
        if (controller == null) {
            controller = super.getPIDController();
        }
        return controller;
    }

    /**
     * Returns a {@link CANAnalog} configured to this {@link TKOSparkMax}
     * Instantiates analog if null
     * @param analogMode The type of {@link CANAnalog.AnalogMode} to set the {@link CANAnalog to}
     * @return analog
     */
    @Override
    public CANAnalog getAnalog(CANAnalog.AnalogMode analogMode) {
        setAnalogMode(analogMode);
        return getAnalog();
    }

    /**
     * Returns a {@link CANAnalog} configured to this {@link TKOSparkMax}
     * Instantiates analog if null
     * If setAnalog is never called, the default {@link CANAnalog.AnalogMode} is kRelative
     * @return analog
     */
    public CANAnalog getAnalog() {
        if (analog == null) {
            analog = super.getAnalog(analogMode);
        }
        return analog;
    }

    /**
     * Configures the encoder if null
     * @param encoderType the {@link EncoderType} to set the encoder to
     * @param cpr the counts per revolution of the encoder
     */
    public void configEncoder(EncoderType encoderType, int cpr) {
        if(encoder == null){
            encoder = super.getEncoder(encoderType, cpr);
            getPIDController().setFeedbackDevice(encoder);
        }
    }

    /**
     * Configures the encoder if null
     * Sets the {@link EncoderType} to kHallSensor and the cpr to 42 (the default of the Spark Max Brushless Encoder)
     */
    public void configEncoder(){
        configEncoder(EncoderType.kHallSensor, 42);
    }

    /**
     * Sets the {@link CANAnalog.AnalogMode} for the {@link CANAnalog} device
     * @param analogMode the {@link CANAnalog.AnalogMode} to set analog mode to
     */
    public void setAnalogMode(CANAnalog.AnalogMode analogMode) {
        this.analogMode = analogMode;
    }

    /**
     * Sets the output of the {@link TKOSparkMax} depending on the control type
     * @param controlType The {@link ControlType} to set the motor to
     * kDutyCycle sets the motor to apply a percentage of it's voltage (1 is 100%, 0.5 is 50%, -1 is 100% backwards)
     *                    This is the same as calling set without specifying the controlType
     * kVelocity sets the motor to use Velocity PID to control the output
     * kVoltage sets the motor to use value amounts of voltage
     * kPosition sets the motor to move to the position value using the PIDController
     * kCurrent sets the motor to use value amounts of amperes
     * @param value the value to set the motor to based on controlType
     */
    public void set(ControlType controlType, double value) {
        getPIDController().setReference(value, controlType);
    }

    /**
     * Configures the PIDF on a certain slotIdx for the {@link TKOSparkMax}
     * @param p       the proportional value
     * @param i       the integral value
     * @param d       the derivative value
     * @param ff      the feed forward value
     * @param slotIdx the slotIdx for the PIDF values
     */
    @Override
    public void configPIDF(double p, double i, double d, double ff, int slotIdx) {
        getPIDController().setP(p, slotIdx);
        getPIDController().setI(i, slotIdx);
        getPIDController().setD(d, slotIdx);
        getPIDController().setFF(ff, slotIdx);
    }

    /**
     * Gets the Raw Encoder Position in ticks
     * @return position in ticks
     */
    @Override
    public double getPositionRaw() {
        return getEncoder().getPosition();
    }

    /**
     * Gets the Raw Encoder Velocity in ticks
     * @return velocity in ticks / minute
     */
    @Override
    public double getVelocityRaw() {
        return getEncoder().getVelocity();
    }

    /**
     * Gets the encoder position with unit conversion
     * The default unit conversion is 1
     * @return encoder position in units
     */
    @Override
    public double getPosition() {
        return getPositionRaw() / ticksPerUnit;
    }

    /**
     * Gets the encoder velocity with unit conversion
     * The default unit conversion is 1
     * @return encoder velocity in units / second
     */
    @Override
    public double getVelocity() {
        return getVelocityRaw() / (60 * ticksPerUnit);
    }

    /**
     * Returns the unit conversion factor for the encoder
     * @return unit conversion factor
     */
    @Override
    public double getTicksPerUnit() {
        return ticksPerUnit;
    }

    /**
     * Sets the unit conversion factor for the encoder
     * @param ticksPerUnit The conversion factor
     */
    @Override
    public void setTicksPerUnit(double ticksPerUnit) {
        if (ticksPerUnit > 0) {
            this.ticksPerUnit = ticksPerUnit;
        }
    }

    /**
     * Inverts the encoder based on the value of inversion
     * @param inversion if the encoder should be inverted
     */
    public void setEncoderInversion(boolean inversion){
        getEncoder().setInverted(inversion);
    }

    /**
     * Returns if the encoder is inverted
     * @return true if inverted
     */
    public boolean getEncoderInverted(){
        return getEncoder().getInverted();
    }

    /**
     * Inverts the encoder from it's current state
     */
    public void invertEncoder(){
        setEncoderInversion(!getEncoderInverted());
    }

    /**
     * Configures the forward roborio limit switch
     * @param id        the id of the forward limit switch
     * @param inversion if the forward limit switch is inverted
     */
    @Override
    public void configRoborioForwardLimitSwitch(int id, boolean inversion) {
        forwardLimitSwitch = new TKODigitalInput(id);
        forwardLimitSwitch.setInverted(inversion);
    }

    /**
     * Configures the reverse roborio limit switch
     * @param id        the id of the reverse limit switch
     * @param inversion if the reverse limit switch is inverted
     */
    @Override
    public void configRoborioReverseLimitSwitch(int id, boolean inversion) {
        reverseLimitSwitch = new TKODigitalInput(id);
        reverseLimitSwitch.setInverted(inversion);
    }

    /**
     * Returns the value of the forward limit switch
     * @return true if the forward limit switch is triggered
     */
    @Override
    public boolean getRoborioForwardLimitSwitch() {
        if (forwardLimitSwitch == null) {
            return false;
        } else {
            return forwardLimitSwitch.get();
        }
    }

    /**
     * Returns the value of the reverse limit switch
     * @return true if the reverse limit switch is triggered
     */
    @Override
    public boolean getRoborioReverseLimitSwitch() {
        if (reverseLimitSwitch == null) {
            return false;
        } else {
            return reverseLimitSwitch.get();
        }
    }

}