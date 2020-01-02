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

package com.github.mittyrobotics.path.following;

import com.github.mittyrobotics.path.following.simulation.PathFollowerSimRobot;
import com.github.mittyrobotics.simulation.rewrite.models.DrivetrainModel;
import com.github.mittyrobotics.simulation.rewrite.motors.CIMMotor;
import com.github.mittyrobotics.simulation.rewrite.sim.RobotSimulator;
import com.github.mittyrobotics.simulation.rewrite.sim.SimDrivetrain;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;

public class Main {

    public static void main(String[] args) {
        PathFollowerSimRobot simRobot =
                new PathFollowerSimRobot(new SimDrivetrain(new DrivetrainModel(50, 1.585, 20, 30,
                        new CIMMotor(), 2, 7, 2)));
        new RobotSimulator(simRobot, 0.02);
    }
}
