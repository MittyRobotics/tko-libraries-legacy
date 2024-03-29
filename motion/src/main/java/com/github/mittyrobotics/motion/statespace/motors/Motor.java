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

package com.github.mittyrobotics.motion.statespace.motors;

/**
 * Motor class.
 * <p>
 * Data for specific motors can be found from: https://motors.vex.com/
 */
public class Motor {

    private final double stallTorque; //Newton meters
    private final double stallCurrent; //Amps
    private final double freeSpeed; //RPM
    private final double freeCurrent; //Amps
    private double numMotors;

    private double resistance;
    private double Kv;
    private double Kt;

    public Motor(double stallTorque, double stallCurrent, double freeSpeed, double freeCurrent, double numMotors) {
        this.stallTorque = stallTorque;
        this.stallCurrent = stallCurrent;
        this.freeSpeed = freeSpeed;
        this.freeCurrent = freeCurrent;
        this.numMotors = numMotors;
        computeMotorValues();
    }

    private void computeMotorValues() {
        this.resistance = 12 / stallCurrent;
        this.Kv = ((freeSpeed / 60.0 * (2.0 * Math.PI)) / (12.0 - resistance * freeCurrent));
        this.Kt = (numMotors * stallTorque) / stallCurrent;
    }

    public double getStallTorque() {
        return stallTorque;
    }

    public double getStallCurrent() {
        return stallCurrent;
    }

    public double getFreeSpeed() {
        return freeSpeed;
    }

    public double getFreeCurrent() {
        return freeCurrent;
    }

    public double getNumMotors() {
        return numMotors;
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
