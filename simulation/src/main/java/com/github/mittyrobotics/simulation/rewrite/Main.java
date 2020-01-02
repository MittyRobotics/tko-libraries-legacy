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

import com.github.mittyrobotics.simulation.rewrite.models.DrivetrainModel;
import com.github.mittyrobotics.simulation.rewrite.motors.CIMMotor;
import com.github.mittyrobotics.simulation.rewrite.sim.RobotSimulator;
import com.github.mittyrobotics.simulation.rewrite.sim.SimDrivetrain;
import com.github.mittyrobotics.simulation.rewrite.sim.SimRobot;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;

public class Main {
    public static void main(String[] args) {
        DrivetrainModel drivetrainModel = new DrivetrainModel(125,1.585,20,30, new CIMMotor(),2,7.0,2);
        SimRobot robot = new SimRobot(new SimDrivetrain(drivetrainModel));

        RobotSimulator simulator = new RobotSimulator(robot,0.02);
        robot.getDrivetrain().setPercentOutput(1,1);
    }
}