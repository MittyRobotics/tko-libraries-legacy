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

package com.github.mittyrobotics.motion.controllers;

import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.motion.DrivetrainWheelSpeeds;

public class DifferentialDriveDynamicsController {
    private final double angularDrag;
    double momentOfInertia;
    double mass;
    double wheelbaseRadius;
    double wheelRadius;
    DCMotorTransmission leftTransmission;
    DCMotorTransmission rightTransmission;

    public DifferentialDriveDynamicsController(double angularDrag, double momentOfInertia, double mass,
                                               double wheelRadius, double wheelbaseRadius,
                                               DCMotorTransmission leftTransmission,
                                               DCMotorTransmission rightTransmission) {
        this.momentOfInertia = momentOfInertia;
        this.mass = mass;
        this.angularDrag = angularDrag;
        this.wheelRadius = wheelRadius;
        this.wheelbaseRadius = wheelbaseRadius;
        this.leftTransmission = leftTransmission;
        this.rightTransmission = rightTransmission;
    }

    public DrivetrainWheelSpeeds calculate(DrivetrainState state) {
        return solveInverseDynamics(state.getVelocity().getLeft(), state.getVelocity().getRight(),
                state.getVelocity().getAngular(), state.getAcceleration().getLinear(),
                state.getAcceleration().getAngular());
    }

    public DrivetrainWheelSpeeds solveInverseDynamics(double leftVelocity, double rightVelocity, double angularVelocity,
                                                      double linearAcceleration, double angularAcceleration) {
        double leftWheelTorque = wheelRadius / 2.0 *
                (linearAcceleration * mass + angularAcceleration * momentOfInertia / wheelbaseRadius -
                        angularVelocity * angularDrag / wheelbaseRadius);

        double rightWheelTorque = wheelRadius / 2.0 *
                (linearAcceleration * mass + angularAcceleration * momentOfInertia / wheelbaseRadius +
                        angularVelocity * angularDrag / wheelbaseRadius);

        double leftVoltage = leftTransmission.getVoltageForTorque(
                leftVelocity, leftWheelTorque);
        double rightVoltage = rightTransmission.getVoltageForTorque(
                rightVelocity, rightWheelTorque);

        return new DrivetrainWheelSpeeds(leftVoltage, rightVoltage);
    }

    public double getAngularDrag() {
        return angularDrag;
    }

    public double getMomentOfInertia() {
        return momentOfInertia;
    }

    public double getMass() {
        return mass;
    }

    public double getWheelbaseRadius() {
        return wheelbaseRadius;
    }

    public double getWheelRadius() {
        return wheelRadius;
    }

    public DCMotorTransmission getLeftTransmission() {
        return leftTransmission;
    }

    public DCMotorTransmission getRightTransmission() {
        return rightTransmission;
    }

    public static class DCMotorTransmission {
        private final double speedPerVolt;
        private final double torquePerVolt;
        private final double vIntercept;

        public static DCMotorTransmission makeTransmission(double kV, double kA, double vIntercept,
                                                           double linearInertia, double wheelRadius) {
            double speedPerVolt = 1 / kV;
            double torquePerVolt = wheelRadius * wheelRadius * linearInertia / (2.0 * kA);
            return new DCMotorTransmission(speedPerVolt, torquePerVolt, vIntercept);
        }

        public DCMotorTransmission(double speedPerVolt, double torquePerVolt, double vIntercept) {
            this.speedPerVolt = speedPerVolt;
            this.torquePerVolt = torquePerVolt;
            this.vIntercept = vIntercept;
        }

        public double getVoltageForTorque(final double output_speed, final double torque) {
            double frictionVoltage;
            if (output_speed > 1e-12 || torque > 1e-12) {
                frictionVoltage = vIntercept;
            } else if (output_speed < -1e-12 || torque < -1e-12) {
                frictionVoltage = -vIntercept;
            } else {
                return 0.0;
            }
            return torque / torquePerVolt + output_speed / speedPerVolt + frictionVoltage;
        }

        public double getSpeedPerVolt() {
            return speedPerVolt;
        }

        public double getTorquePerVolt() {
            return torquePerVolt;
        }

        public double getvIntercept() {
            return vIntercept;
        }
    }
}
