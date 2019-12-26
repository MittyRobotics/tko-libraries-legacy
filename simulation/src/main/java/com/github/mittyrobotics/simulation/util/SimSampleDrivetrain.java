/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.simulation.util;

import com.github.mittyrobotics.simulation.motors.CIMMotor;
import com.github.mittyrobotics.simulation.sim.ModelSystem;
import com.github.mittyrobotics.simulation.sim.RobotSimManager;
import com.github.mittyrobotics.simulation.sim.SimDrivetrain;
import com.github.mittyrobotics.simulation.sim.SimTalon;

public class SimSampleDrivetrain extends SimDrivetrain {
    private static SimSampleDrivetrain instance = new SimSampleDrivetrain();
    private double p;
    private double i;
    private double d;
    private double f;
    private double maxPIDPercent = 12;
    private double lastMeasured;
    private double lastError;

    public static SimSampleDrivetrain getInstance() {
        return instance;
    }

    @Override
    public void initDrivetrain() {
        SimTalon[] leftTalons = new SimTalon[]{
                new SimTalon(new ModelSystem(new CIMMotor())),
                new SimTalon(new ModelSystem(new CIMMotor()))
        };
        SimTalon[] rightTalons = new SimTalon[]{
                new SimTalon(new ModelSystem(new CIMMotor())),
                new SimTalon(new ModelSystem(new CIMMotor()))
        };

        setupSimDriveTalons(leftTalons, rightTalons);

        leftTalons[1].setFollower(leftTalons[0]);
        rightTalons[1].setFollower(rightTalons[0]);
    }

    public void setSpeeds(double left, double right) {
        getLeftMasterTalon().set(left);
        getRightMasterTalon().set(right);
    }

    public void setVelocities(double leftVelocity, double rightVelocity) {
        getLeftMasterTalon().set(calculatePID(leftVelocity, getLeftMasterTalon().getVelocity(),
                RobotSimManager.getInstance().getPeriodTime()));
        getRightMasterTalon().set(calculatePID(rightVelocity, getRightMasterTalon().getVelocity(),
                RobotSimManager.getInstance().getPeriodTime()));
    }

    public void setupPIDFValues(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    private double calculatePID(double target, double measured, double deltaTime) {
        double voltage = 0;

        double FF = f * target + d * ((measured - lastMeasured) / deltaTime);

        double error = target - measured;

        double FB = p * error;

        voltage = FF + FB;

        double maxVoltage = maxPIDPercent * 12;

        voltage = Math.max(-maxVoltage, Math.min(maxVoltage, voltage));

        lastMeasured = measured;
        lastError = error;

        return voltage / 12;
    }

    public double getP() {
        return p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double getI() {
        return i;
    }

    public void setI(double i) {
        this.i = i;
    }

    public double getD() {
        return d;
    }

    public void setD(double d) {
        this.d = d;
    }

    public double getF() {
        return f;
    }

    public void setF(double f) {
        this.f = f;
    }

    public double getMaxPIDPercent() {
        return maxPIDPercent;
    }

    public void setMaxPIDPercent(double maxPIDPercent) {
        this.maxPIDPercent = maxPIDPercent;
    }
}
