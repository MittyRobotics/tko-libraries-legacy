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

package com.github.mittyrobotics.motion.controllers;

public class PIDFController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double period;

    private double previousError;
    private double error;
    private double derivativeError;
    private double integral;

    private double minIntegral;
    private double maxIntegral;

    private double minOutput;
    private double maxOutput;

    private double setpoint;
    private double positionTolerance;
    private double derivativeTolerance;

    public PIDFController(double kP, double kI, double kD, double kF, double period) {
        setGains(kP, kI, kD, kF);
        setPeriod(period);
        setPositionTolerance(Double.POSITIVE_INFINITY);
        setDerivativeTolerance(Double.POSITIVE_INFINITY);
        setOutputRange(-1, 1);
        setIntegralRange(-1, 1);
    }

    public PIDFController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0.02);
    }

    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    /**
     * Calculates the next voltage for the PIDF controller given a value measurement and a delta time since the last
     * <code>calculate()</code> call.
     *
     * @param measurement current measured value.
     * @param deltaTime   time since last <code>calculate()</code> call.
     * @return next voltage for PIDF controller.
     */
    public double calculate(double measurement, double deltaTime) {
        return calculate(measurement, 0, deltaTime);
    }

    /**
     * Calculates the next voltage for the PIDF controller given a value measurement, a feedforward voltage, and a delta
     * time since the last <code>calculate()</code> call.
     *
     * @param measurement current measured value.
     * @param feedForward the feedforward voltage to be applied to the controller.
     * @param deltaTime   time since last <code>calculate()</code> call.
     * @return next voltage for PIDF controller.
     */
    public double calculate(double measurement, double feedForward, double deltaTime) {
        return calculate(measurement, feedForward, setpoint - measurement, deltaTime);
    }

    public double calculate(double measurement, double feedForward, double error, double deltaTime) {
        double feedback = calculateFeedback(measurement, error, deltaTime);

        //Output feedback plus feed forward
        return Math.min(maxOutput, Math.max(feedback + feedForward, minOutput));
    }

    private double calculateFeedback(double measurement, double error, double deltaTime) {
        //Set the current period to the delta time
        setPeriod(deltaTime);
        //Keep track of previous error
        previousError = error;
        //Set derivative error
        derivativeError = (error - previousError) / period;

        //Update integral if there is an I term
        if (kI != 0) {
            integral = Math.max(minIntegral / kI, Math.min(integral + error * period, maxIntegral / kI));
        }

        //Calculate feedback
        return kP * error + kI * integral + kD * derivativeError;
    }

    /**
     * Calculates the next voltage for the PIDF controller given a value measurement. Delta time is set to default
     * delta time.
     *
     * @param measurement current measured value.
     * @return next voltage for PIDF controller.
     */
    public double calculate(double measurement) {
        return calculate(measurement, period);
    }

    /**
     * Resets the controller.
     */
    public void reset() {
        previousError = 0;
        integral = 0;
    }

    public boolean isFinished(double positionTolerance, double derivativeTolerance) {
        return Math.abs(error) < positionTolerance && Math.abs(derivativeError) < derivativeTolerance;
    }

    public boolean isFinished(double positionTolerance) {
        return isFinished(positionTolerance, derivativeTolerance);
    }

    public boolean isFinished() {
        return isFinished(positionTolerance, derivativeTolerance);
    }

    /**
     * Sets the integral range. Default -1 to 1.
     *
     * @param minIntegral minimum integral value
     * @param maxIntegral maximum integral value
     */
    public void setIntegralRange(double minIntegral, double maxIntegral) {
        this.minIntegral = minIntegral;
        this.maxIntegral = maxIntegral;
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }


    public void setD(double kD) {
        this.kD = kD;
    }

    public void setF(double kF) {
        this.kF = kF;
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        setP(kP);
        setI(kI);
        setD(kD);
        setF(kF);
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getkF() {
        return kF;
    }

    public double getPeriod() {
        return period;
    }

    public void setPeriod(double period) {
        this.period = period;
    }

    public double getError() {
        return error;
    }

    public double getDerivativeError() {
        return derivativeError;
    }

    public double getPreviousError() {
        return previousError;
    }

    public double getMinIntegral() {
        return minIntegral;
    }

    public double getMaxIntegral() {
        return maxIntegral;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public void setPositionTolerance(double tolerance) {
        positionTolerance = tolerance;
    }

    public double getDerivativeTolerance() {
        return derivativeTolerance;
    }

    public void setDerivativeTolerance(double tolerance) {
        derivativeTolerance = tolerance;
    }

}