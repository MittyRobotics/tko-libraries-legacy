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

import com.github.mittyrobotics.datatypes.motion.DrivetrainWheelSpeeds;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.statespace.motors.Motor;

public class DrivetrainModel {
    private final double mass;
    private final double momentOfInertia;
    private final double trackWidth;
    private final double drivetrainLength;
    private final Motor motor;
    private final double numMotorsPerSide;
    private final double gearRatio;
    private final double wheelRadius;

    private double resistance;
    private double Kv;
    private double Kt;

    private double leftVelocity;
    private double rightVelocity;
    private double leftAcceleration;
    private double rightAcceleration;
    private double leftPosition;
    private double rightPosition;

    /**
     * @param mass            lbs
     * @param momentOfInertia kg*m^2
     * @param trackWidth      in
     * @param motor
     * @param gearRatio
     * @param wheelRadius     in
     */
    public DrivetrainModel(double mass, double momentOfInertia, double trackWidth, double drivetrainLength,
                           Motor motor,
                           double gearRatio, double wheelRadius) {
        this.mass = mass * Conversions.LBS_TO_KG;
        this.momentOfInertia = momentOfInertia;
        this.trackWidth = trackWidth * Conversions.IN_TO_M;
        this.drivetrainLength = drivetrainLength;
        this.motor = motor;
        this.numMotorsPerSide = motor.getNumMotors();
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius * Conversions.IN_TO_M;
        computeModelValues();
    }

    private void computeModelValues() {
        double stallTorque = motor.getStallTorque();
        double stallCurrent = motor.getStallCurrent();
        double freeSpeed = motor.getFreeSpeed();
        double freeCurrent = motor.getFreeCurrent();

        this.resistance = 12.0 / stallCurrent;
        this.Kv = ((freeSpeed / 60.0 * 2.0 * Math.PI) / (12.0 - resistance * freeCurrent));
        this.Kt = (numMotorsPerSide * stallTorque) / stallCurrent;
    }

    public void updateModel(double leftVoltage, double rightVoltage, double deltaTime) {
        DrivetrainWheelSpeeds accelerations = calculateAccelerations(leftVoltage, rightVoltage);
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
    public DrivetrainWheelSpeeds calculateAccelerations(double leftVoltage, double rightVoltage) {
        double Vl = leftVoltage;
        double Vr = rightVoltage;
        double vl = leftVelocity;
        double vr = rightVelocity;
        double m = mass;
        double rb = trackWidth;
        double J = momentOfInertia;
        double G = gearRatio;
        double R = resistance;
        double r = wheelRadius;
        double c1 = -(G * G * Kt) / (Kv * R * (r * r));
        double c2 = (G * Kt) / (R * r);

        double Fl = (c1 * vl + c2 * Vl);
        double Fr = (c1 * vr + c2 * Vr);

        double eqn0 = (1 / m) + ((rb * rb) / J);
        double eqn1 = (1 / m) - ((rb * rb) / J);

        double leftAcceleration = eqn0 * Fl + eqn1 * Fr;
        double rightAcceleration = eqn1 * Fl + eqn0 * Fr;

        return new DrivetrainWheelSpeeds(leftAcceleration, rightAcceleration);
    }

    public double getTrackWidth() {
        return trackWidth * Conversions.M_TO_IN;
    }

    public double getDrivetrainLength() {
        return drivetrainLength;
    }

    public double getLeftVelocity() {
        return leftVelocity * Conversions.M_TO_IN;
    }

    public double getRightVelocity() {
        return rightVelocity * Conversions.M_TO_IN;
    }

    public double getLeftAcceleration() {
        return leftAcceleration * Conversions.M_TO_IN;
    }

    public double getRightAcceleration() {
        return rightAcceleration * Conversions.M_TO_IN;
    }

    public double getLeftPosition() {
        return leftPosition * Conversions.M_TO_IN;
    }

    public double getRightPosition() {
        return rightPosition * Conversions.M_TO_IN;
    }
}
