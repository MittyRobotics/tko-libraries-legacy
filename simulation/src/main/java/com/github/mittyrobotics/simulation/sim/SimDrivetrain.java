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

package com.github.mittyrobotics.simulation.sim;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motion.modeling.models.DrivetrainModel;

import java.util.Timer;
import java.util.TimerTask;

public class SimDrivetrain extends TimerTask {
    private final DrivetrainModel drivetrainModel;
    private double leftVoltage;
    private double rightVoltage;
    private double periodTime = 0.001;

    //Odometry values
    private Transform robotTransform = new Transform();
    private double prevLeftPos;
    private double prevRightPos;

    //PID values
    private double p;
    private double i;
    private double d;
    private double f;
    private double lastMeasured;
    private double lastError;
    private double maxPIDPercent = 1;

    public SimDrivetrain(DrivetrainModel drivetrainModel) {
        this.drivetrainModel = drivetrainModel;
        new Timer().scheduleAtFixedRate(this, (long) 0.0, (long) (periodTime * 1000.0));
    }

    public void setPercentOutput(double leftPercent, double rightPercent) {
        leftPercent = Math.max(-1, Math.min(1, leftPercent));
        rightPercent = Math.max(-1, Math.min(1, rightPercent));
        this.leftVoltage = leftPercent * 12;
        this.rightVoltage = rightPercent * 12;
    }

    public void setVelocityControl(double leftVelocity, double rightVelocity) {
        setPercentOutput(calculatePID(leftVelocity, drivetrainModel.getLeftVelocity(), periodTime),
                calculatePID(rightVelocity, drivetrainModel.getRightVelocity(), periodTime));
    }

    private void odometry() {
        double deltaLeftPos = (drivetrainModel.getLeftPosition() - prevLeftPos);
        double deltaRightPos = (drivetrainModel.getRightPosition() - prevRightPos) ;

        double deltaPos = (deltaLeftPos + deltaRightPos) / 2;

        Rotation rotation =
                robotTransform.getRotation()
                        .subtract(Rotation.fromRadians(Math.atan2((deltaLeftPos - deltaRightPos),
                                drivetrainModel.getTrackWidth())));

        Position position = robotTransform.getPosition().add(new Position(rotation.cos() * deltaPos,
                rotation.sin() * deltaPos));

        robotTransform = new Transform(position, rotation);

        prevLeftPos = drivetrainModel.getLeftPosition();
        prevRightPos = drivetrainModel.getRightPosition();
    }

    public void setOdometry(Transform robotTransform) {
        this.robotTransform = robotTransform;

        prevLeftPos = drivetrainModel.getLeftPosition();
        prevRightPos = drivetrainModel.getRightPosition();
    }

    public Transform getRobotTransform() {
        return robotTransform;
    }

    public void setupPIDFValues(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public void setMaxPIDPercent(double maxPIDPercent) {
        this.maxPIDPercent = maxPIDPercent;
    }

    private double calculatePID(double target, double measured, double deltaTime) {
        double voltage;

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

    @Override
    public void run() {
        drivetrainModel.updateModel(leftVoltage, rightVoltage, periodTime);
        odometry();
    }

    public DrivetrainModel getDrivetrainModel() {
        return drivetrainModel;
    }

    public double getLeftVoltage() {
        return leftVoltage;
    }

    public double getRightVoltage() {
        return rightVoltage;
    }
}
