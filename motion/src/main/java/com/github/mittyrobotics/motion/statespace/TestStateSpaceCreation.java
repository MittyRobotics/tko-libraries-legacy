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

package com.github.mittyrobotics.motion.statespace;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.motion.OverrideMethod;
import com.github.mittyrobotics.motion.SCurveMotionProfile;
import com.github.mittyrobotics.motion.statespace.models.PulleyModel;
import com.github.mittyrobotics.motion.statespace.motors.CIMMotor;
import com.github.mittyrobotics.motion.statespace.motors.Motor;
import com.github.mittyrobotics.visualization.MotorGraph;
import org.ejml.simple.SimpleMatrix;

public class TestStateSpaceCreation {
    public static void main(String[] args) {
        double pulleyRadius = .1;
        double mass = 6.803886;
        double gearReduction = 42.0 / 12.0 * 40.0 / 14.0;
        double maxVoltage = 12;
        double dt = 0.00505;
        Motor motor = new CIMMotor(2);

        Plant plant = Plant.createElevatorPlant(motor, mass, pulleyRadius, gearReduction,
                maxVoltage, dt);
        KalmanFilter filter = new KalmanFilter(plant,
                new SimpleMatrix(new double[][]{{0.05, 1.0}}),
                new SimpleMatrix(new double[][]{{0.0001}}));
        LinearQuadraticRegulator controller = new LinearQuadraticRegulator(plant,
                new SimpleMatrix(new double[][]{{0.02, 0.4}}), new SimpleMatrix(new double[][]{{12.0}}));

        System.out.println("GAINS \n" + plant.getDiscreteSystem().getA() + "" + plant.getDiscreteSystem().getB() + "" +
                plant.getDiscreteSystem().getC() + "" + plant.getDiscreteSystem().getD() + "" + controller.getLqrGain() + "" +
                controller.getUff() + "" + filter.getKalmanGain());

        StateSpaceLoop loop = new StateSpaceLoop(plant, controller, filter);

        MotorGraph graph = new MotorGraph();

        PulleyModel pulleyModel = new PulleyModel(mass, motor, gearReduction, pulleyRadius);

        SCurveMotionProfile motionProfile = new SCurveMotionProfile(new MotionState(0, 0, 0), new MotionState(5, 0, 0),
                1, 1, 1, 5, OverrideMethod.END_AFTER_SETPOINT);


        double previousPos = 0;
        double previousVel = 0;
        for (double t = 0; t < 10; t += dt) {
            double referencePosition = motionProfile.calculateState(t).getPosition();
            double referenceVelocity = motionProfile.calculateState(t).getVelocity();
            loop.setNextR(new SimpleMatrix(new double[][]{{5}, {0}}));
            loop.correct(new SimpleMatrix(new double[][]{{previousPos}}));
            loop.predict(dt);
            double voltage = loop.getU().get(0);
            pulleyModel.updateModel(voltage, dt);
            previousPos = pulleyModel.getPosition();
            previousVel = pulleyModel.getVelocity();
            graph.addPosition(previousPos, t);
            graph.addVelocity(previousVel, t);
            graph.addVoltage(voltage, t);
            graph.addError(loop.getError(), t);
//            Thread.sleep(200);
        }
    }
}
