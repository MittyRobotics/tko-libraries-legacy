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
        PIDFController controller = new PIDFController(60, 0, 60);
//        controller.setIntegralRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        controller.setPeriod(DELTA_T);
//        controller.setOutputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        controller.setSetpoint(setpoint);
        double position = 0;
        double thePrevPos;
        double t = 0;
        while (t < 10) {
            if((int)(t* 1000) % (int)(1000 * DELTA_T) == 0){
                thePrevPos = position;
                final double MAX_MOTOR_ACCELERATION = .1;
                if(Math.abs(velocity - controller.calculate(thePrevPos)) < MAX_MOTOR_ACCELERATION){
                    velocity = controller.calculate(thePrevPos);
                } else if(velocity < controller.calculate(thePrevPos)){
                    velocity += MAX_MOTOR_ACCELERATION;
                } else {
                    velocity -= MAX_MOTOR_ACCELERATION;
                }
            }
            graph.addPosition(position, t);
            graph.addVelocity(velocity, t);
            position += VELOCITY_PER_PERCENT * velocity * GRAPH_UPDATE_TIME;
            t += GRAPH_UPDATE_TIME;
        }
    }
}

