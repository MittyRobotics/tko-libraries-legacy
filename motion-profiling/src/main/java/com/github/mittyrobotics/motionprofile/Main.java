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

package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.visualization.MotorGraph;

public class Main {
    public static void main(String[] args) {
        DynamicSCurveMotionProfile motionProfile = new DynamicSCurveMotionProfile(50, 50, 50, 13,
                OverrideMethod.OVERSHOOT);
        DynamicTrapezoidalMotionProfile motionProfile1 = new DynamicTrapezoidalMotionProfile(10, 20, 20,
                OverrideMethod.OVERSHOOT);
        MotorGraph graph = new MotorGraph("S-curve Motion Profile", "position (m), velocity (m/s), acceleration " +
                "(m/s^2)", "time (s)");
//        MotorGraph graph1 = new MotorGraph();
        MotionState currentState = new MotionState(0, 0, 0);
        MotionState currentState1 = new MotionState(0, 0, 0);
        MotionState desiredState = new MotionState(15, 0, 0);
        double t = -100;
        double deltaT = .01;
        double m = 20;
        double b = 10;
        double x0 = 0;
        double v0 = 0;
        while (t < 100) {
//            double position = motionProfile.calculateNextState(currentState, desiredState, deltaT).getPosition();
//            double velocity = motionProfile.calculateNextState(currentState, desiredState, deltaT).getVelocity();
//            double acceleration =
//                    motionProfile.calculateNextState(currentState, desiredState, deltaT).getAcceleration();
//            double temp =
//                    motionProfile.calculateNextState(currentState, desiredState, deltaT).getTemp();
//            graph.addPosition(position, t);
//            graph.addVelocity(velocity, t);
//            graph.addAcceleration(acceleration, t);
//            graph.addSetpoint(temp, t);
//            currentState = new MotionState(position, velocity, acceleration);
//            double position1 = motionProfile1.calculateNextState(currentState1, desiredState, deltaT).getPosition();
//            double velocity1 = motionProfile1.calculateNextState(currentState1, desiredState, deltaT).getVelocity();
//            double acceleration1 =
//                    motionProfile1.calculateNextState(currentState1, desiredState, deltaT).getAcceleration();
//
////            graph.addError(velocity1,t);
////            graph.addVoltage(acceleration1,t);
//            currentState1 = new MotionState(position1, velocity1, acceleration1);

            Position[] points = motionProfile.epicEquation(t,m,b,x0,v0);
            for(Position p : points){
                graph.addPosition(p.getY(),p.getX());
            }
            t += deltaT;
        }
    }
}
