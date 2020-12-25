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

package com.github.mittyrobotics.motion.modeling.models;

import com.github.mittyrobotics.datatypes.motion.DrivetrainWheelState;
import com.github.mittyrobotics.motion.modeling.motors.Motor;

public class DrivetrainModel {
    private final double mass;
    private final double momentOfInertia;
    private final double trackWidth;
    private final double drivetrainLength;
    private final Motor motor;
    private final double gearRatio;
    private final double wheelRadius;

    private double leftVelocity;
    private double rightVelocity;
    private double leftAcceleration;
    private double rightAcceleration;
    private double leftPosition;
    private double rightPosition;

    /**
     * @param mass
     * @param momentOfInertia kg*m^2
     * @param trackWidth
     * @param motor
     * @param gearRatio
     * @param wheelRadius
     */
    public DrivetrainModel(double mass, double momentOfInertia, double trackWidth, double drivetrainLength,
                           Motor motor,
                           double gearRatio, double wheelRadius) {
        this.mass = mass;
        this.momentOfInertia = momentOfInertia;
        this.trackWidth = trackWidth;
        this.drivetrainLength = drivetrainLength;
        this.motor = motor;
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
    }

    public void updateModel(double leftVoltage, double rightVoltage, double deltaTime) {
        DrivetrainWheelState accelerations = calculateAccelerations(leftVoltage, rightVoltage);

        this.leftAcceleration = accelerations.getLeft();
        this.leftVelocity += leftAcceleration * deltaTime;
        this.leftPosition += leftVelocity * deltaTime;

        this.rightAcceleration = accelerations.getRight();
        this.rightVelocity += rightAcceleration * deltaTime;
        this.rightPosition += rightVelocity * deltaTime;
    }

    /**
     * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#page=191
     *
     * @param leftVoltage
     * @param rightVoltage
     */
    private DrivetrainWheelState calculateAccelerations(double leftVoltage, double rightVoltage) {
        double Vl = leftVoltage;
        double Vr = rightVoltage;
        double vl = leftVelocity;
        double vr = rightVelocity;
        double m = mass;
        double rb = trackWidth;
        double J = momentOfInertia;
        double G = gearRatio;
        double R = motor.getResistance();
        double r = wheelRadius;
        double Kt = motor.getKt();
        double Kv = motor.getKv();

        double c1 = -(G * G * Kt) / (Kv * R * (r * r));
        double c2 = (G * Kt) / (R * r);

        double Fl = (c1 * vl + c2 * Vl);
        double Fr = (c1 * vr + c2 * Vr);

        double eqn0 = (1 / m) + ((rb * rb) / J);
        double eqn1 = (1 / m) - ((rb * rb) / J);

        double leftAcceleration = eqn0 * Fl + eqn1 * Fr;
        double rightAcceleration = eqn1 * Fl + eqn0 * Fr;

        return new DrivetrainWheelState(leftAcceleration, rightAcceleration);
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getDrivetrainLength() {
        return drivetrainLength;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public double getLeftAcceleration() {
        return leftAcceleration;
    }

    public double getRightAcceleration() {
        return rightAcceleration;
    }

    public double getLeftPosition() {
        return leftPosition;
    }

    public double getRightPosition() {
        return rightPosition;
    }
}
