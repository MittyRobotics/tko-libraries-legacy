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

package com.github.mittyrobotics.simulation.rewrite;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.simulation.rewrite.models.DrivetrainModel;
import com.github.mittyrobotics.simulation.rewrite.models.PulleyModel;
import com.github.mittyrobotics.simulation.rewrite.motors.CIMMotor;
import com.github.mittyrobotics.simulation.rewrite.motors.Pro775Motor;
import com.github.mittyrobotics.simulation.sim.ModelSystem;
import com.github.mittyrobotics.visualization.graphs.MotorSimGraph;

public class Main {
    public static void main(String[] args) {
        DrivetrainModel drivetrainModel = new DrivetrainModel(125,1.585,20, new CIMMotor(),2,7,2);

        MotorSimGraph graph = new MotorSimGraph();

        double voltage = 12;
        for(int i = 0; i < 500; i++){
            drivetrainModel.updateModel(voltage,voltage,0.02);
            if(i > 200){
                voltage = 0;
            }
            graph.addAcceleration((drivetrainModel.getLeftAcceleration()+drivetrainModel.getRightAcceleration())/2,i*0.02);
            graph.addVelocity((drivetrainModel.getLeftVelocity()+drivetrainModel.getRightVelocity())/2,i*0.02);
            graph.addPosition((drivetrainModel.getLeftPosition()+drivetrainModel.getRightPosition())/2,i*0.02);
        }
    }
}
