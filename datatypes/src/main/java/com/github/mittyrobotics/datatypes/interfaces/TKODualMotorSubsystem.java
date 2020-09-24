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
//public abstract class TKODualMotorSubsystem extends TKOSubsystem {
//
//    public TKODualMotorSubsystem(String name) {
//        super(name);
//    }
//
//    abstract void overrideSetMotor(double leftPercent, double rightPercent);
//
//    public void setMotor(double leftPercent, double rightPercent) {
//        overrideSetMotor(leftPercent, rightPercent);
//    }
//
//    public void stopMotor() {
//        overrideSetMotor(0, 0);
//    }
//
//    public void resetEncoder() {
//
//    }
//
//    public double getLeftPosition() {
//        return 0;
//    }
//
//    public double getLeftVelocity() {
//        return 0;
//    }
//
//    public double getRightPosition() {
//        return 0;
//    }
//
//    public double getRightVelocity() {
//        return 0;
//    }
//
//    public double getAveragePosition() {
//        return (getLeftPosition() + getRightPosition()) / 2;
//    }
//
//    public double getAverageVelocity() {
//        return (getLeftVelocity() + getRightVelocity()) / 2;
//    }
//
//    public boolean getLeftSwitch() {
//        return false;
//    }
//
//    public boolean getRightSwitch() {
//        return false;
//    }
//}
