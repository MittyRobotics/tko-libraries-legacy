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

import edu.wpi.first.wpilibj.controller.PIDController;

public class PIDFController extends PIDController {
    private double Kf;
    private ControlType controlType;
    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period, ControlType controlType) {
        super(Kp, Ki, Kd, period);
        setF(Kf);
        setControlType(controlType);
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, ControlType controlType){
        this(Kp, Ki, Kd, Kf, 0.02, controlType);
    }

    public PIDFController(double Kp, double Ki, double Kd){
        this(Kp, Ki, Kd, 0, ControlType.Position);
    }

    public void setF(double Kf){
        this.Kf = Kf;
    }

    public double getF(){
        return Kf;
    }

    public void setControlType(ControlType controlType){
        this.controlType = controlType;
    }

    public ControlType getControlType(){
        return controlType;
    }

    @Override
    public double calculate(double measurement) {
        switch (controlType){
            case Position:
                return super.calculate(measurement) + Kf;
            case Velocity:
                return super.calculate(measurement) + super.getSetpoint() * Kf;
        }
        return super.calculate(measurement);
    }

    public enum ControlType {
        Position, Velocity
    }
}