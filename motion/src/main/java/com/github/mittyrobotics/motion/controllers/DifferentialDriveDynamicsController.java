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

import com.github.mittyrobotics.datatypes.motion.DrivetrainSpeeds;
import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.motion.DrivetrainWheelSpeeds;

/**
 * Controller that calculates feedforward voltages to achieve a {@link DrivetrainState} from the dynamics of a
 * differential drive.
 * <p>
 * Dynamics equations from Team 254's DifferentialDrive physics class:
 * https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/lib/physics/DifferentialDrive.java
 */
public class DifferentialDriveDynamicsController {
    private final double angularDrag;
    double momentOfInertia;
    double mass;
    double wheelbaseRadius;
    double wheelRadius;
    DCMotorTransmission leftTransmission;
    DCMotorTransmission rightTransmission;

    /**
     * Creates a new {@link DifferentialDriveDynamicsController} with the drivetrain properties.
     *
     * @param angularDrag
     * @param momentOfInertia
     * @param mass
     * @param wheelRadius
     * @param trackWidth
     * @param leftTransmission
     * @param rightTransmission
     */
    public DifferentialDriveDynamicsController(double angularDrag, double momentOfInertia, double mass,
                                               double wheelRadius, double trackWidth,
                                               DCMotorTransmission leftTransmission,
                                               DCMotorTransmission rightTransmission) {
        this.momentOfInertia = momentOfInertia;
        this.mass = mass;
        this.angularDrag = angularDrag;
        this.wheelRadius = wheelRadius;
        this.wheelbaseRadius = trackWidth / 2;
        this.leftTransmission = leftTransmission;
        this.rightTransmission = rightTransmission;
    }

    /**
     * Calculates feedforward voltages to achieve the desired {@link DrivetrainState} <code>state</code>.
     *
     * @param state Desired state to achieve
     * @return feedforward voltages in {@link DrivetrainWheelSpeeds}
     */
    public DrivetrainWheelSpeeds calculate(DrivetrainState state) {
        return solveInverseDynamics(state.getVelocity().getLeft(), state.getVelocity().getRight(),
                state.getVelocity().getAngular(), state.getAcceleration().getLinear(),
                state.getAcceleration().getAngular());
    }

    public DrivetrainState solveForwardDynamics(DrivetrainState currentState, DrivetrainWheelSpeeds voltages) {
        if (Math.abs(currentState.getVelocity().getLeft()) < 1e-12 &&
                Math.abs(currentState.getVelocity().getRight()) < 1e-12 &&
                Math.abs(voltages.getLeft()) < leftTransmission.getVIntercept() &&
                Math.abs(voltages.getRight()) < rightTransmission.getVIntercept()) {
            return new DrivetrainState(DrivetrainSpeeds.fromLinearAndAngular(0, 0,
                    wheelbaseRadius * 2), DrivetrainSpeeds.fromLinearAndAngular(0, 0,
                    wheelbaseRadius * 2));
        }
        double leftTorque =
                leftTransmission.getTorqueForVoltage(currentState.getVelocity().getLeft(), voltages.getLeft());
        double rightTorque = rightTransmission.getTorqueForVoltage(currentState.getVelocity().getRight(),
                voltages.getRight());

        double linearAcceleration = (rightTorque + leftTorque) / (wheelRadius * mass);
        double angularAcceleration =
                wheelbaseRadius * (rightTorque - leftTorque) / (wheelRadius * momentOfInertia) -
                        currentState.getVelocity().getAngular() * angularDrag / momentOfInertia;

        return new DrivetrainState(currentState.getVelocity(),
                DrivetrainSpeeds.fromLinearAndAngular(linearAcceleration, angularAcceleration,
                        wheelbaseRadius * 2));
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

    /**
     * Represents a DC motor transmission on a drivetrain. This can be thought of as a model of the dynamics of a
     * side of the drivetrain, in which torque can be solved for voltage or voltage solved for torque. Used in the
     * {@link DifferentialDriveDynamicsController} to compute the voltage for a desired {@link DrivetrainState}.
     *
     * Equations from Team 254's DCMotorTransmission physics class:
     * https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/lib/physics/DCMotorTransmission.java
     */
    public static class DCMotorTransmission {
        private final double speedPerVolt;
        private final double torquePerVolt;
        private final double vIntercept;

        /**
         * Builds a {@link DCMotorTransmission} from some measurable properties of the drivetrain.
         *
         * @param kV          voltage constant, V per rad/s
         * @param kA          acceleration constant, V per rad/s^2
         * @param vIntercept  friction voltage, the voltage required to overcome frictional forces.
         * @param mass        mass of drivetrain.
         * @param wheelRadius radius of drivetrain wheels.
         * @return a calculated {@link DCMotorTransmission}.
         */
        public static DCMotorTransmission makeTransmission(double kV, double kA, double vIntercept,
                                                           double mass, double wheelRadius) {
            double speedPerVolt = 1 / kV;
            double torquePerVolt = wheelRadius * wheelRadius * mass / (2.0 * kA);
            return new DCMotorTransmission(speedPerVolt, torquePerVolt, vIntercept);
        }

        /**
         * Represents a DC motor transmission on a drivetrain.
         *
         * @param speedPerVolt
         * @param torquePerVolt
         * @param vIntercept
         */
        public DCMotorTransmission(double speedPerVolt, double torquePerVolt, double vIntercept) {
            this.speedPerVolt = speedPerVolt;
            this.torquePerVolt = torquePerVolt;
            this.vIntercept = vIntercept;
        }

        /**
         * Computes the torque for a given velocity and voltage.
         *
         * @param velocity the transmission's current wheel velocity
         * @param voltage  the transmission's current voltage
         * @return the torque
         */
        public double getTorqueForVoltage(final double velocity, final double voltage) {
            double effectiveVoltage = voltage;
            if (velocity > 1e-12) {
                effectiveVoltage -= vIntercept;
            } else if (velocity < -1e-12) {
                effectiveVoltage += vIntercept;
            } else if (voltage > 1e-12) {
                effectiveVoltage = Math.max(0.0, voltage - vIntercept);
            } else if (voltage < -1e-12) {
                effectiveVoltage = Math.min(0.0, voltage + vIntercept);
            } else {
                return 0.0;
            }
            return torquePerVolt * (-velocity / speedPerVolt + effectiveVoltage);
        }

        /**
         * Computes the voltage for a given velocity and torque.
         *
         * @param velocity the transmission's current wheel velocity
         * @param torque   the transmission's current torque
         * @return the voltage
         */
        public double getVoltageForTorque(final double velocity, final double torque) {
            double frictionVoltage;
            if (velocity > 1e-12 || torque > 1e-12) {
                frictionVoltage = vIntercept;
            } else if (velocity < -1e-12 || torque < -1e-12) {
                frictionVoltage = -vIntercept;
            } else {
                return 0.0;
            }
            return torque / torquePerVolt + velocity / speedPerVolt + frictionVoltage;
        }

        public double getSpeedPerVolt() {
            return speedPerVolt;
        }

        public double getTorquePerVolt() {
            return torquePerVolt;
        }

        public double getVIntercept() {
            return vIntercept;
        }
    }
}
