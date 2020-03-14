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

package com.github.mittyrobotics.simulation.models;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.simulation.motors.Motor;

public class FlywheelModel {
    private final Motor motor;
    private final double numMotors;
    private final double gearRatio;
    private final double momentOfInertia;

    private double resistance;
    private double Kv;
    private double Kt;

    private double angularAcceleration;
    private double angularVelocity;
    private double torque;

    public FlywheelModel(double momentOfInertia, Motor motor, double numMotors, double gearRatio) {
        this.momentOfInertia = momentOfInertia;
        this.motor = motor;
        this.numMotors = numMotors;
        this.gearRatio = gearRatio;
        computeModelValues();
    }

    public void computeModelValues() {
        double stallTorque = motor.getStallTorque();
        double stallCurrent = motor.getStallCurrent();
        double freeSpeed = motor.getFreeSpeed();
        double freeCurrent = motor.getFreeCurrent();

        this.resistance = 12 / stallCurrent;
        this.Kv = ((freeSpeed / 60.0 * 2.0 * Math.PI) / (12.0 - resistance * freeCurrent));
        this.Kt = (numMotors * stallTorque) / stallCurrent;
    }

    public void updateModel(double voltage, double deltaTime) {
        this.angularAcceleration = calculateAcceleration(voltage);
        this.angularVelocity += angularAcceleration * deltaTime;
        this.torque = calculateTorque(this.angularAcceleration);
    }

    private double calculateTorque(double angularAcceleration){
        double wf = angularAcceleration;
        double J = momentOfInertia;
        return J*wf;
    }

    private double calculateAcceleration(double voltage) {
        double G = gearRatio;
        double R = resistance;
        double V = voltage;
        double w = angularVelocity;
        double J = momentOfInertia;

        return (G * Kt) / (R*J) * V - (G * G * Kt) / (Kv*R*J) * w;
    }

    public double getAngularAcceleration() {
        return angularAcceleration * Conversions.M_TO_IN;
    }

    public double getAngularVelocity() {
        return angularVelocity * Conversions.M_TO_IN;
    }

    public double getTorque(){
        return torque;
    }
}
