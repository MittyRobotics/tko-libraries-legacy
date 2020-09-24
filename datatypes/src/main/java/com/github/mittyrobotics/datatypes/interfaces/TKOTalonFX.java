///*
// * MIT License
// *
// * Copyright (c) 2020 Mitty Robotics (Team 1351)
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// *
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package com.github.mittyrobotics.datatypes.interfaces;
//
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//
///**
// * Enhanced {@link WPI_TalonFX} with extra features
// */
//public class TKOTalonFX extends WPI_TalonFX implements PIDFInterface, LimitSwitchInterface, TicksConversionInterface {
//
//    /**
//     * Limit switches to be used if they are wired through the roborio
//     * If the limit switches are not wired through the roborio, use getSensorPhase() instead
//     */
//    private TKODigitalInput forwardLimitSwitch, reverseLimitSwitch;
//
//    /**
//     * The conversion factor to convert encoder ticks to other units
//     */
//    private double ticksPerUnit;
//
//    /**
//     * Constructs a {@link TKOTalonFX} object
//     *
//     * @param deviceNumber the id of the {@link WPI_TalonFX}
//     */
//    public TKOTalonFX(int deviceNumber) {
//        super(deviceNumber);
//        configFactoryDefault();
//    }
//
//    /**
//     * Configures the forward roborio limit switch
//     *
//     * @param id        the id of the forward limit switch
//     * @param inversion if the forward limit switch is inverted
//     */
//    @Override
//    public void configRoborioForwardLimitSwitch(int id, boolean inversion) {
//        forwardLimitSwitch = new TKODigitalInput(id);
//        forwardLimitSwitch.setInverted(inversion);
//    }
//
//    /**
//     * Configures the reverse roborio limit switch
//     *
//     * @param id        the id of the reverse limit switch
//     * @param inversion if the reverse limit switch is inverted
//     */
//    @Override
//    public void configRoborioReverseLimitSwitch(int id, boolean inversion) {
//        reverseLimitSwitch = new TKODigitalInput(id);
//        reverseLimitSwitch.setInverted(inversion);
//    }
//
//    /**
//     * Returns the value of the forward limit switch
//     *
//     * @return true if the forward limit switch is triggered
//     */
//    @Override
//    public boolean getRoborioForwardLimitSwitch() {
//        if (forwardLimitSwitch == null) {
//            return false;
//        } else {
//            return forwardLimitSwitch.get();
//        }
//    }
//
//    /**
//     * Returns the value of the reverse limit switch
//     *
//     * @return true if the reverse limit switch is triggered
//     */
//    @Override
//    public boolean getRoborioReverseLimitSwitch() {
//        if (reverseLimitSwitch == null) {
//            return false;
//        } else {
//            return reverseLimitSwitch.get();
//        }
//    }
//
//    /**
//     * Configures the PIDF on a certain slotIdx for the {@link TKOTalonFX}
//     *
//     * @param p       the proportional value
//     * @param i       the integral value
//     * @param d       the derivative value
//     * @param ff      the feed forward value
//     * @param slotIdx the slotIdx for the PIDF values
//     */
//    @Override
//    public void configPIDF(double p, double i, double d, double ff, int slotIdx) {
//        super.config_kP(slotIdx, p);
//        super.config_kI(slotIdx, p);
//        super.config_kD(slotIdx, p);
//        super.config_kF(slotIdx, p);
//    }
//
//    /**
//     * Gets the Raw Encoder Position in ticks
//     *
//     * @return position in ticks
//     */
//    @Override
//    public double getPositionRaw() {
//        return getSelectedSensorPosition();
//    }
//
//    /**
//     * Gets the Raw Encoder Velocity in ticks
//     *
//     * @return velocity in ticks / 100ms
//     */
//    @Override
//    public double getVelocityRaw() {
//        return getSelectedSensorVelocity();
//    }
//
//    /**
//     * Gets the encoder position with unit conversion
//     * The default unit conversion is 1
//     *
//     * @return encoder position in units
//     */
//    @Override
//    public double getPosition() {
//        return getPositionRaw() / ticksPerUnit;
//    }
//
//    /**
//     * Gets the encoder velocity with unit conversion
//     * The default unit conversion is 1
//     *
//     * @return encoder velocity in units / second
//     */
//    @Override
//    public double getVelocity() {
//        return getVelocityRaw() / ticksPerUnit * 10;
//    }
//
//    /**
//     * Returns the unit conversion factor for the encoder
//     *
//     * @return unit conversion factor
//     */
//    @Override
//    public double getTicksPerUnit() {
//        return ticksPerUnit;
//    }
//
//    /**
//     * Sets the unit conversion factor for the encoder
//     *
//     * @param ticksPerUnit The conversion factor
//     */
//    @Override
//    public void setTicksPerUnit(double ticksPerUnit) {
//        if (ticksPerUnit > 0) {
//            this.ticksPerUnit = ticksPerUnit;
//        }
//    }
//}