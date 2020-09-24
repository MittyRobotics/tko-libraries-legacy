/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion.statespace.models;

import com.github.mittyrobotics.motion.statespace.motors.Motor;

public class SingleJointedArmModel {
    private Motor motor;
    private double gearReduction;
    private double momentOfInertia;

    private double angle = 0;
    private double angularVelocity = 0;
    private double angularAcceleration = 0;

    public SingleJointedArmModel(Motor motor, double gearReduction, double momentOfInertia) {
        this.motor = motor;
        this.gearReduction = gearReduction;
        this.momentOfInertia = momentOfInertia;
    }

    public void update(double voltage, double dt) {
        this.angularAcceleration = calculateAngularAcceleration(voltage);
        this.angularVelocity += this.angularAcceleration * dt;
        this.angle += this.angularVelocity * dt;
    }

    private double calculateAngularAcceleration(double voltage) {
        return (gearReduction * gearReduction * motor.getKt()) /
                (motor.getKv() * motor.getResistance() * momentOfInertia) * angularVelocity +
                (gearReduction * motor.getKt()) / (motor.getResistance() * momentOfInertia) * voltage;
    }

    public Motor getMotor() {
        return motor;
    }

    public double getGearReduction() {
        return gearReduction;
    }

    public double getMomentOfInertia() {
        return momentOfInertia;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getAngularAcceleration() {
        return angularAcceleration;
    }
}
