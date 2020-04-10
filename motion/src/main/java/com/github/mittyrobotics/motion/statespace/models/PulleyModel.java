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

package com.github.mittyrobotics.motion.statespace.models;

import com.github.mittyrobotics.motion.statespace.motors.Motor;

public class PulleyModel {
    private final double mass;
    private final Motor motor;
    private final double gearRatio;
    private final double pulleyRadius;

    private double resistance;
    private double Kv;
    private double Kt;

    private double acceleration;
    private double velocity;
    private double position;

    public PulleyModel(double mass, Motor motor, double gearRatio, double pulleyRadius) {
        this.mass = mass;
        this.motor = motor;
        this.gearRatio = gearRatio;
        this.pulleyRadius = pulleyRadius;
        computeModelValues();
    }

    public void computeModelValues() {
        double stallTorque = motor.getStallTorque();
        double stallCurrent = motor.getStallCurrent();
        double freeSpeed = motor.getFreeSpeed();
        double freeCurrent = motor.getFreeCurrent();

        this.resistance = motor.getResistance();
        this.Kv = motor.getKv();
        this.Kt = motor.getKt();
    }

    public void updateModel(double voltage, double deltaTime) {
        this.acceleration = calculateAcceleration(voltage);
        this.velocity += acceleration * deltaTime;
        this.position += velocity * deltaTime;
    }

    private double calculateAcceleration(double voltage) {
        double G = gearRatio * 100;
        double R = resistance * 100;
        double r = pulleyRadius * 100;
        double m = mass * 100;
        double V = voltage * 100;
        double v = velocity * 100;
        double Kt = this.Kt * 100;
        double Kv = this.Kv * 100;

        return ((G * Kt) / (R * r * m) * V - (G * G * Kt) / (R * (r * r) * m * Kv) * v) * 100;
    }

    public double getAcceleration() {
        return acceleration / 100;
    }

    public double getVelocity() {
        return velocity / 100;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity * 100;
    }

    public double getPosition() {
        return position / 100;
    }

    public void setPosition(double position) {
        this.position = position * 100;
    }

    public double getMass() {
        return mass;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getPulleyRadius() {
        return pulleyRadius;
    }

    public double getResistance() {
        return resistance;
    }

    public double getKv() {
        return Kv;
    }

    public double getKt() {
        return Kt;
    }
}
