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

package com.github.mittyrobotics.simulation;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.modeling.models.DrivetrainModel;
import com.github.mittyrobotics.motion.modeling.motors.CIMMotor;
import com.github.mittyrobotics.simulation.sim.PathFollowerSimRobot;
import com.github.mittyrobotics.simulation.sim.RobotSimulator;
import com.github.mittyrobotics.simulation.sim.SimDrivetrain;
import com.github.mittyrobotics.visualization.RobotGraph;

public class Main {
    public static void main(String[] args) {
        DrivetrainModel drivetrainModel =
                new DrivetrainModel(50 * Conversions.LBS_TO_KG, 1.585, 20 * Conversions.IN_TO_M,
                        30 * Conversions.IN_TO_M, new CIMMotor(2), 7.0, 2 * Conversions.IN_TO_M);
        PathFollowerSimRobot robot = new PathFollowerSimRobot(new SimDrivetrain(drivetrainModel));

        RobotSimulator simulator = new RobotSimulator(robot, 0.02, new RobotGraph());
    }
}
