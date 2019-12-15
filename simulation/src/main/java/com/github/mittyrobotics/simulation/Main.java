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
		
		MotorSimulator motorSimulator = new MotorSimulator(new CIMMotor(), 2, mass, gearRatio, wheelRadius, controlLoop, controlType, "CIM motor");
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
