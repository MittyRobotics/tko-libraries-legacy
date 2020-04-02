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

package com.github.mittyrobotics.datatypes.interfaces;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TKOTalonFX extends WPI_TalonFX implements PIDInterface, LimitSwitchInterface {
    //Use these if you want to have limit switches but they are wired through the roborio
    private TKODigitalInput forwardLimitSwitch, reverseLimitSwitch;

    public TKOTalonFX(int deviceNumber) {
        super(deviceNumber);
        configFactoryDefault();
    }

    public void configRoborioForwardLimitSwitch(int id, boolean inversion) {
        forwardLimitSwitch = new TKODigitalInput(id);
        forwardLimitSwitch.setInverted(inversion);
    }

    public void configRoborioReverseLimitSwitch(int id, boolean inversion) {
        reverseLimitSwitch = new TKODigitalInput(id);
        reverseLimitSwitch.setInverted(inversion);
    }

    public boolean getRoborioForwardLimitSwitch() {
        if (forwardLimitSwitch == null) {
            return false;
        } else {
            return forwardLimitSwitch.get();
        }
    }

    public boolean getRoborioReverseLimitSwitch() {
        if (reverseLimitSwitch == null) {
            return false;
        } else {
            return reverseLimitSwitch.get();
        }
    }

    @Override
    public void configPIDF(double p, double i, double d, double ff, int slotIdx) {
        super.config_kP(slotIdx, p);
        super.config_kI(slotIdx, i);
        super.config_kD(slotIdx, d);
        super.config_kF(slotIdx, ff);
    }
}