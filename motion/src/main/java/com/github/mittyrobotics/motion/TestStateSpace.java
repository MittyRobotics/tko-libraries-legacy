/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.statespace.LinearQuadraticRegulator;
import com.github.mittyrobotics.motion.statespace.LuenbergerObserver;
import com.github.mittyrobotics.motion.statespace.StateSpace;
import com.github.mittyrobotics.motion.statespace.StateSpaceSystem;
import com.github.mittyrobotics.motion.statespace.models.FlywheelModel;
import com.github.mittyrobotics.motion.statespace.motors.Motor;
import com.github.mittyrobotics.motion.statespace.motors.NEOMotor;
import com.github.mittyrobotics.motion.statespace.motors.Pro775Motor;
import com.github.mittyrobotics.visualization.MotorGraph;
import org.ejml.simple.SimpleMatrix;
import org.jblas.util.Random;

public class TestStateSpace {
    public static void main(String[] args) throws InterruptedException {
        double gearReduction = 1;
        Motor motor = new NEOMotor(2);
        double momentOfInertia = 0.005;

        SimpleMatrix states, outputs, a, b, c, d;
        states = new SimpleMatrix(new double[1][1]); //[[angular velocity]]
        states.zero();
        outputs = new SimpleMatrix(new double[1][1]); //[[angular velocity]]
        outputs.zero();

        double G = gearReduction;
        double Kt = motor.getKt();
        double Kv = motor.getKv();
        double R = motor.getResistance();
        double J = momentOfInertia;

        a = new SimpleMatrix(new double[1][1]);
        a.fill(-(G * G * Kt) / (Kv * R * J));

        b = new SimpleMatrix(new double[1][1]);
        b.fill((G * Kt) / (R * J));

        c = SimpleMatrix.identity(1);

        d = new SimpleMatrix(new double[1][1]);
        d.zero();

        double dt = 0.02;

        StateSpaceSystem system = new StateSpaceSystem(a, b, c, d);
        StateSpaceSystem systemD = system.sample(dt);

        //State space state matrix
        SimpleMatrix x = new SimpleMatrix(1, 1);
        //State space measurement matrix
        SimpleMatrix y = new SimpleMatrix(1, 1);

        //Observer state estimate matrix
        SimpleMatrix xHat = new SimpleMatrix(1, 1);
        //Obserever measurement estimate matrix
        SimpleMatrix yHat = new SimpleMatrix(1, 1);

        double velocityTolerance = 9.42;
        double voltageTolerance = 12;
        double qWeight = 0.01;

        SimpleMatrix K = LinearQuadraticRegulator
                .linearQuadraticRegulator(systemD, new SimpleMatrix(new double[][]{{velocityTolerance}}),
                        new SimpleMatrix(new double[][]{{voltageTolerance}}), qWeight);
        SimpleMatrix Kff = StateSpace.calculateFeedforwardGains(systemD);

        SimpleMatrix L = new SimpleMatrix(new double[][]{{1}});

        MotorGraph motorGraph = new MotorGraph();

        FlywheelModel model = new FlywheelModel(motor, momentOfInertia, gearReduction, 12);

        SimpleMatrix r = new SimpleMatrix(new double[][]{{400}});
        for (double t = 0; t < 10; t += dt) {
            xHat = LuenbergerObserver.correctX(systemD, xHat, L, y, yHat);
            yHat = LuenbergerObserver.correctY(systemD, xHat);

            SimpleMatrix u = StateSpace.closedLoopControlLaw(xHat, r, K, StateSpace.MAX_VOLTAGE);
            SimpleMatrix uff = StateSpace.closedLoopFeedforwardControlLaw(systemD, r, Kff, StateSpace.MAX_VOLTAGE);
            u = StateSpace.applyFeedforward(u, StateSpace.MAX_VOLTAGE, uff);

            model.updateModel(u.get(0), dt);

            y = new SimpleMatrix(new double[][]{{model.getAngularVelocity()}});

            xHat = LuenbergerObserver.predict(systemD, xHat, u);

            motorGraph.addVoltage(u.get(0), t);
            motorGraph.addSetpoint(r.get(0), t);
            motorGraph.addVelocity(yHat.get(0), t);
        }
        model = new FlywheelModel(motor, momentOfInertia, gearReduction, 12);
    }
}
