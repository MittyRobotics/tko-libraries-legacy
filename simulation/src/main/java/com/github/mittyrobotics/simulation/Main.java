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


import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.simulation.enums.ControlLoopType;
import com.github.mittyrobotics.simulation.enums.ControlType;
import com.github.mittyrobotics.simulation.motors.CIMMotor;
import com.github.mittyrobotics.simulation.sim.ControlLoop;
import com.github.mittyrobotics.simulation.sim.MotorSimulator;
import com.github.mittyrobotics.visualization.graphs.MotorSimGraph;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;


public class Main {

    public static void main(String[] args) {
        double mass = 40.0 * Conversions.LBS_TO_KG; //Kg
        double gearRatio = 42.0 / 11.0 * 50.0 / 24.0;
        double wheelRadius = 2 * Conversions.IN_TO_M; //Meters
        double maxVoltage = 12;

        double iterationTime = 0.02;

        ControlLoop controlLoop = new ControlLoop(ControlLoopType.VELOCITY, maxVoltage, iterationTime);
        controlLoop.setupVelocityController(1, 0, 0);
        ControlType controlType = ControlType.VELOCITY;

        MotorSimulator motorSimulator =
                new MotorSimulator(new CIMMotor(), 2, mass, gearRatio, wheelRadius, controlLoop, controlType,
                        "CIM motor");
        MotorSimGraph graph = new MotorSimGraph();

        double setpoint = 10 * Conversions.IN_TO_M;

        double t = 0.0;


        while (t < 20) {
//			if (t % 5.0 <= iterationTime) {
//				setpoint = -setpoint;
//			}
            motorSimulator.update(setpoint, iterationTime);
            graph.addPosition(motorSimulator.getPosition(), t);
            graph.addVelocity(motorSimulator.getVelocity(), t);
            graph.addVoltage(motorSimulator.getVoltage(), t);
            graph.addSetpoint(setpoint, t);
            t += iterationTime;
        }
    }

    private static void writeToCSV(List<List<String>> rows) throws IOException {
        FileWriter csvWriter = new FileWriter("data.csv");
        csvWriter.append("Time");
        csvWriter.append(",");
        csvWriter.append("Input");
        csvWriter.append(",");
        csvWriter.append("Output");
        csvWriter.append("\n");

        for (List<String> rowData : rows) {
            csvWriter.append(String.join(",", rowData));
            csvWriter.append("\n");
        }

        csvWriter.flush();
        csvWriter.close();
    }
}
