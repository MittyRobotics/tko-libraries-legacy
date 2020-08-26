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
import com.github.mittyrobotics.motion.statespace.models.FlywheelModel;
import com.github.mittyrobotics.motion.statespace.models.PulleyModel;
import com.github.mittyrobotics.motion.statespace.motors.Motor;
import com.github.mittyrobotics.motion.statespace.motors.NEOMotor;
import com.github.mittyrobotics.visualization.MotorGraph;
import org.ejml.simple.SimpleMatrix;

public class TestStateSpaceCreation {
    public static void main(String[] args) throws InterruptedException {
        double momentOfInertia = 0.002634;
        double gearReduction = 1;
        double maxVoltage = 3;
        double dt = 0.001;
        Motor motor = new NEOMotor(2);

        double modelVelocityAccuracyGain = 1.0; //Model velocity accuracy
        double measurementAccuracyGain = 0.01; //Accuracy of measurement, encoder accuracy
        double velocityTolerance = 0.4;
        double voltageTolerance = 12;
        //Weights the Q matrix in the LQR for different response according to Bryson's Rule. Lower Q results in a more
        //gradual response, higher Q results in a more aggressive response. Default of 1.
        double qWeight = 0.0000001;

        Plant plant = Plant.createFlywheelPlant(motor, momentOfInertia, gearReduction, maxVoltage, dt);

        KalmanFilter observer = new KalmanFilter(plant,
                new SimpleMatrix(new double[][]{{modelVelocityAccuracyGain}}),
                new SimpleMatrix(new double[][]{{measurementAccuracyGain}}));

        LinearQuadraticRegulator controller = new LinearQuadraticRegulator(plant,
                new SimpleMatrix(new double[][]{{velocityTolerance}}),
                new SimpleMatrix(new double[][]{{voltageTolerance}}), qWeight);

        StateSpaceController loop = new StateSpaceController(plant, controller, observer);

        MotorGraph graph = new MotorGraph("State Space Elevator Motion Profile", "Position (x10), Velocity(x10), " +
                "Voltage", "Time");

        FlywheelModel model = new FlywheelModel(motor, momentOfInertia, gearReduction, maxVoltage);

        model.setMeasurementNoise(.0);

        double previousVel = 0;

        plant.setX(new SimpleMatrix(new double[][]{{previousVel}}));
        model.setAngularVelocity(previousVel);
        for (double t = 0; t < 10; t += dt) {
            double referenceVelocity = 1000 * Conversions.RPM_TO_RAD_PER_SECOND;
            if (t < 1) {
                referenceVelocity = 0;
            }


            SimpleMatrix voltage = loop.calculate(new SimpleMatrix(new double[][]{{previousVel}}),
                    new SimpleMatrix(new double[][]{{referenceVelocity}}), dt);

            model.updateModel(voltage.get(0), dt);

            previousVel = model.getAngularVelocity();

            graph.addVelocity(previousVel * Conversions.RAD_PER_SECOND_TO_RPM, t);
            graph.addVoltage(voltage.get(0), t);
//            graph.addError(loop.getError() * 10, t);
            graph.addSetpoint(referenceVelocity * Conversions.RAD_PER_SECOND_TO_RPM, t);
//            graph.addAcceleration(elevatorModel.getAcceleration(), t);

            Thread.sleep((long) (dt*1000));
        }
    }
}
