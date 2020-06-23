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

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.controllers.StateSpaceController;
import com.github.mittyrobotics.motion.profiles.OverrideMethod;
import com.github.mittyrobotics.motion.profiles.SCurveMotionProfile;
import com.github.mittyrobotics.motion.statespace.KalmanFilter;
import com.github.mittyrobotics.motion.statespace.LinearQuadraticRegulator;
import com.github.mittyrobotics.motion.statespace.Plant;
import com.github.mittyrobotics.motion.statespace.models.PulleyModel;
import com.github.mittyrobotics.motion.statespace.motors.Motor;
import com.github.mittyrobotics.motion.statespace.motors.NEOMotor;
import com.github.mittyrobotics.visualization.MotorGraph;
import org.ejml.simple.SimpleMatrix;

public class TestStateSpaceCreation {
    public static void main(String[] args) {
        double pulleyRadius = 1.5 * Conversions.IN_TO_M;
        double mass = 20 * Conversions.LBS_TO_KG;
        double gearReduction = 10;
        double maxVoltage = 12;
        double dt = 0.001;
        Motor motor = new NEOMotor(2);

        double modelPositionAccuracyGain = 0.05; //Model position accuracy
        double modelVelocityAccuracyGain = 1.0; //Model velocity accuracy
        double measurementAccuracyGain = 0.01; //Accuracy of measurement, encoder accuracy
        double positionTolerance = 0.02;
        double velocityTolerance = 0.4;
        double voltageTolerance = 12;
        //Weights the Q matrix in the LQR for different response according to Bryson's Rule. Lower Q results in a more
        //gradual response, higher Q results in a more aggressive response. Default of 1.
        double qWeight = 0.01;

        Plant plant = Plant.createElevatorPlant(motor, mass, pulleyRadius, gearReduction,
                maxVoltage, dt);

        KalmanFilter observer = new KalmanFilter(plant,
                new SimpleMatrix(new double[][]{{modelPositionAccuracyGain, modelVelocityAccuracyGain}}),
                new SimpleMatrix(new double[][]{{measurementAccuracyGain}}));

        LinearQuadraticRegulator controller = new LinearQuadraticRegulator(plant,
                new SimpleMatrix(new double[][]{{positionTolerance, velocityTolerance}}),
                new SimpleMatrix(new double[][]{{voltageTolerance}}), qWeight);

        StateSpaceController loop = new StateSpaceController(plant, controller, observer);

        MotorGraph graph = new MotorGraph("State Space Elevator Motion Profile", "Position (x10), Velocity(x10), " +
                "Voltage", "Time");

        PulleyModel elevatorModel = new PulleyModel(motor, mass, gearReduction, pulleyRadius, 12);

        elevatorModel.setMeasurementNoise(.0);

        double previousPos = 0;
        double previousVel = 0;

        SCurveMotionProfile motionProfile = new SCurveMotionProfile(new MotionState(previousPos, previousVel, 0),
                new MotionState(previousPos + 1, 0, 0),
                50, 50, 100, 2, OverrideMethod.END_AFTER_SETPOINT);

        plant.setX(new SimpleMatrix(new double[][]{{previousPos}, {previousVel}}));
        elevatorModel.setPosition(previousPos);
        elevatorModel.setVelocity(previousVel);
        for (double t = 0; t < 5; t += dt) {
            double referencePosition = 1;
            if (t < 1) {
                referencePosition = 0;
            }

            double referenceVelocity = 0;

            SimpleMatrix voltage = loop.calculate(new SimpleMatrix(new double[][]{{previousPos}}),
                    new SimpleMatrix(new double[][]{{referencePosition}, {referenceVelocity}}), dt);

            elevatorModel.updateModel(voltage.get(0), dt);

            previousPos = elevatorModel.getPosition();
            previousVel = elevatorModel.getVelocity();

            graph.addPosition(previousPos * 10, t);
            graph.addVelocity(previousVel * 10, t);
            graph.addVoltage(voltage.get(0), t);
//            graph.addError(loop.getError() * 10, t);
            graph.addSetpoint(referencePosition * 10, t);
            graph.addAcceleration(elevatorModel.getAcceleration(), t);
        }
    }
}
