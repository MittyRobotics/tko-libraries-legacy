/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.simulation;

import com.github.mittyrobotics.simulation.motors.NEOMotor;
import com.github.mittyrobotics.simulation.models.FlywheelModel;
import com.github.mittyrobotics.visualization.graphs.MotorSimGraph;

public class MotorSimTest {
    public static void main(String[] args) {
        FlywheelModel model = new FlywheelModel(0.04115, new NEOMotor(),1,1);
        MotorSimGraph graph = new MotorSimGraph();
        double voltage = 6;
        for(double i = 0; i < 50; i+=0.1){
            model.updateModel(voltage,0.1);
            //graph.addVelocity(model.getAngularVelocity(),i);
            graph.addVoltage(voltage, i);
            graph.addError(model.getTorque(), i);
        }
    }
}
