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
        DrivetrainModel drivetrainModel = new DrivetrainModel(50,1.463198267146,20, new CIMMotor(),2,7,2);
        PulleyModel pulleyModel = new PulleyModel(20,new Pro775Motor(),1,7,1);
        ModelSystem model = new ModelSystem(new Pro775Motor());
        model.initSystemModel(44*Conversions.LBS_TO_KG,7,1*Conversions.IN_TO_M);
        MotorSimGraph graph = new MotorSimGraph();
        MotorSimGraph graph1 = new MotorSimGraph();
        for(int i = 0; i < 500; i++){
            drivetrainModel.updateModel(12,12,0.02);
            graph.addAcceleration((drivetrainModel.getLeftAcceleration()+drivetrainModel.getRightAcceleration())/2,i*0.02);
            graph.addVelocity((drivetrainModel.getLeftVelocity()+drivetrainModel.getRightVelocity())/2,i*0.02);
            graph.addPosition((drivetrainModel.getLeftPosition()+drivetrainModel.getRightPosition())/2,i*0.02);
        }
//
//        for(int i = 0; i < 500; i++){
//            pulleyModel.updateModel(12,0.02);
//            graph.addAcceleration(pulleyModel.getAcceleration(),i*0.02);
//            graph.addVelocity(pulleyModel.getVelocity(),i*0.02);
//            graph.addPosition(pulleyModel.getPosition(),i*0.02);
//        }
        for(int i = 0; i < 500; i++){
            model.updateModel(12,0.02);
            graph1.addAcceleration(model.getAcceleration()* Conversions.M_TO_IN,i*0.02);
            graph1.addVelocity(model.getVelocity()* Conversions.M_TO_IN,i*0.02);
            graph1.addPosition(model.getPosition()* Conversions.M_TO_IN,i*0.02);
        }
    }
}
