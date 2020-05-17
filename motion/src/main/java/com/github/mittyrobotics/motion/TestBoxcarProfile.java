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

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.motion.profiles.BoxcarFilterProfile;
import com.github.mittyrobotics.visualization.MotorGraph;

public class TestBoxcarProfile {
    public static void main(String[] args) {
        MotorGraph graph = new MotorGraph();
        double dt = 0.001;

        BoxcarFilterProfile profile = new BoxcarFilterProfile(100, 40, .2, .1, dt);

        for(double t = 0; t < 5; t += dt){
            MotionState state = profile.calculate();
            graph.addPosition(state.getPosition(), t);
            graph.addVelocity(state.getVelocity(), t);
            graph.addAcceleration(state.getAcceleration(), t);

        }
    }
}
