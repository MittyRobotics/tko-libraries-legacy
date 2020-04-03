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

import com.github.mittyrobotics.motion.controllers.ElevatorStateSpaceController;
import com.github.mittyrobotics.visualization.MotorGraph;
import org.ejml.simple.SimpleMatrix;

public class ControllerTesting {
    public static void main(String[] args) {
        MotorGraph graph = new MotorGraph();
        SimpleMatrix A = new SimpleMatrix(new double[][]{
                {1.0, 0.0034323689390278016},
                {0.0, 0.4363739579808415}
        });
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {0.00021137763582757403},
                {0.07364963688398798}
        });
        SimpleMatrix C = new SimpleMatrix(new double[][]{
                {1.0, 0.0}
        });
        SimpleMatrix D = new SimpleMatrix(new double[][]{
                {0.0}
        });
        SimpleMatrix K = new SimpleMatrix(new double[][]{
                {232.76812610676953, 5.540693882702348}
        });
        SimpleMatrix Kff = new SimpleMatrix(new double[][]{
                {0.03896854934387037, 13.577687619768485}
        });
        SimpleMatrix kalmanGain = new SimpleMatrix(new double[][]{
                {0.9999960231492777},
                {0.7347579419051207}
        });
        ElevatorStateSpaceController controller = new ElevatorStateSpaceController(-1, 0, 1, 0, A, B, C, D, K, Kff,
                kalmanGain);
        double measuredPosition = 0;
        double measuredVelocity = 0;
        for(double t = 0; t < 10; t+=0.00505){
            double v = controller.calculate(measuredPosition);
            graph.addVoltage(v, t);
            measuredPosition = controller.getEstimatedPosition();
            measuredVelocity = controller.getEstimatedVelocity();
            graph.addPosition(measuredPosition,t);
            graph.addVelocity(measuredVelocity,t);
        }
    }
}
