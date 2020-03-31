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
import com.github.mittyrobotics.visualization.MotorGraph;

import javax.swing.*;

public class Main extends JFrame {
    private MotionState currentState = new MotionState(0, 0, 0);
    private MotionState desiredState = new MotionState(20, 0, 0);
    private MotorGraph graph;

    public static void main(String[] args) {
        new Main().start();
    }

    public void start() {
        this.graph = new MotorGraph("S-curve Motion Profile", "position (m), velocity (m/s), acceleration " +
                "(m/s^2)", "time (s)");
        calcProfile();
    }

    public void calcProfile() {
        SCurveMotionProfile motionProfile = new SCurveMotionProfile(currentState, desiredState, 20, 20, 50, 20, OverrideMethod.OVERSHOOT);
        double t = 0;
        double deltaT = .01;
        while (t < 10) {
            MotionState state = motionProfile.calculateState(t);
            double position = state.getPosition();
            double velocity = state.getVelocity();
            double acceleration = state.getAcceleration();
            graph.addPosition(position, t);
            graph.addVelocity(velocity, t);
            graph.addAcceleration(acceleration, t);

            t += deltaT;
        }
    }
}

