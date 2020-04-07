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

import com.github.mittyrobotics.visualization.MotorGraph;

import javax.swing.*;

public class PIDGrapher extends JFrame {
    private MotorGraph graph;

    public static void main(String[] args) {
        new PIDGrapher().start();
    }

    public void start() {
        this.graph = new MotorGraph("PID Graph", "position (m)", "time (s)");
        calcProfile();
    }

    public void calcProfile() {
        double velocity = 0;
        final double DELTA_T = .02;
        final double GRAPH_UPDATE_TIME = .001;
        double setpoint = 5;
        final double VELOCITY_PER_PERCENT = 1;
        PIDFController controller = new PIDFController(5 / 1.7, .5 / 2, 1.0 / 8);
//        controller.setIntegralRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        controller.setPeriod(DELTA_T);
//        controller.setOutputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        controller.setSetpoint(setpoint);
        double position = 0;
        double thePrevPos = 0;
        double t = 0;
        double acceleration = 0;
        double theVelocity = 0;
        while (t < 10) {
            if ((int) (t * 1000) % (int) (1000 * DELTA_T) == 0) {
                thePrevPos = position;
                theVelocity = velocity;
                final double MAX_MOTOR_ACCELERATION = .1;
                if (Math.abs(velocity - controller.calculate(thePrevPos)) < MAX_MOTOR_ACCELERATION) {
                    velocity = controller.calculate(thePrevPos);
                    acceleration = controller.calculate(thePrevPos) - velocity;
                } else if (velocity < controller.calculate(thePrevPos)) {
                    velocity += MAX_MOTOR_ACCELERATION;
                    acceleration = MAX_MOTOR_ACCELERATION;
                } else {
                    velocity -= MAX_MOTOR_ACCELERATION;
                    acceleration = -MAX_MOTOR_ACCELERATION;
                }
            }
            graph.addPosition(position, t);
            graph.addVelocity(theVelocity, t);
            graph.addSetpoint(setpoint, t);
            graph.addError(setpoint - position, t);

            position += VELOCITY_PER_PERCENT * velocity * GRAPH_UPDATE_TIME;
            theVelocity += acceleration * GRAPH_UPDATE_TIME / DELTA_T;
            t += GRAPH_UPDATE_TIME;
        }
    }
}

