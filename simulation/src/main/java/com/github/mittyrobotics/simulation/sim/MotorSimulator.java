package com.github.mittyrobotics.simulation.sim;


import com.github.mittyrobotics.simulation.enums.ControlType;
import com.github.mittyrobotics.simulation.motors.Motor;
import com.github.mittyrobotics.visualization.graphs.MotorSimGraph;

/**
 * Simulates a mechanism given data on the motors that drive it.
 * <p>
 * The motor modeling calculations are from 971's series about control system modeling and test driven development:
 * https://www.youtube.com/watch?v=uGtT8ojgSzg
 * https://www.youtube.com/watch?v=RLrZzSpHP4E
 */

public class MotorSimulator {
	
	private Motor motor;
	private double numMotors;
	private double mass; //kg
	private double gearRatio;
	private double wheelRadius; //m
	
	private double Kv; //velocity constant
	private double Kt; //torque constant
	private double resistance;
	
	private ControlLoop controlLoop;
	private ControlType controlType;
	private String name;
	private double velocity = 0;
	private double position = 0;
	private double voltage = 0;
	/**
	 * Motor Simulator Constructor
	 * <p>
	 * Initiates the properties of the motor simulator.
	 *
	 * @param motor
	 * @param numMotors
	 * @param mass
	 * @param gearRatio
	 * @param wheelRadius
	 * @param controlLoop
	 * @param controlType
	 */
	public MotorSimulator(Motor motor, double numMotors, double mass, double gearRatio, double wheelRadius, ControlLoop controlLoop, ControlType controlType) {
		this(motor, numMotors, mass, gearRatio, wheelRadius, controlLoop, controlType, "Motor Sim");
	}
	/**
	 * Motor Simulator Constructor
	 * <p>
	 * Initiates the properties of the motor simulator.
	 *
	 * @param motor
	 * @param numMotors
	 * @param mass
	 * @param gearRatio
	 * @param wheelRadius
	 * @param controlLoop
	 * @param controlType
	 * @param name
	 */
	public MotorSimulator(Motor motor, double numMotors, double mass, double gearRatio, double wheelRadius, ControlLoop controlLoop, ControlType controlType, String name) {
		this.motor = motor;
		this.numMotors = numMotors;
		this.mass = mass;
		this.gearRatio = gearRatio;
		this.wheelRadius = wheelRadius;
		this.controlLoop = controlLoop;
		this.controlType = controlType;
		this.name = name;

		double stallTorque = motor.getStallTorque();
		double stallCurrent = motor.getStallCurrent();
		double freeSpeed = motor.getFreeSpeed();
		double freeCurrent = motor.getFreeCurrent();
		this.resistance = 12 / stallCurrent;
		this.Kv = ((freeSpeed / 60.0 * 2.0 * Math.PI) / (12.0 - resistance * freeCurrent));
		this.Kt = (numMotors * stallTorque) / stallCurrent;
	}
	
	/**
	 * Iterates through the motor model and graphs it over time
	 *
	 * @param setpoint
	 * @param simulationTime
	 * @param iterationTime
	 */
	public void graphModel(double setpoint, double simulationTime, double iterationTime) {
		MotorSimGraph graph = new MotorSimGraph();
		
		double threshold = 0.1;
		
		double t = 0.0;
		
		
		while (t < simulationTime) {
			update(setpoint, iterationTime);
			graph.addPosition(position, t);
			graph.addVelocity(velocity, t);
			graph.addVoltage(voltage, t);
			graph.addSetpoint(setpoint, t);
			t += iterationTime;
		}
	}
	
	/**
	 * Updates the motor model.
	 * <p>
	 * Calculates the voltage from a {@link ControlLoop} and gets the applies the acceleration calculation to it.
	 *
	 * @param setpoint
	 * @param iterationTime
	 */
	public void update(double setpoint, double iterationTime) {
		if (controlType == ControlType.POSITION) {
			voltage = controlLoop.update(setpoint, position);
		} else if (controlType == ControlType.VELOCITY) {
			voltage = controlLoop.update(setpoint, velocity);
		} else {
			voltage = 0;
		}
		
		double acceleration = getAcceleration(voltage);
		position += velocity * iterationTime;
		velocity += acceleration * iterationTime;
	}
	
	/**
	 * Calculates acceleration from a given voltage following the input motor model.
	 *
	 * @param voltage
	 * @return
	 */
	private double getAcceleration(double voltage) {
		return -Kt * gearRatio * gearRatio / (Kv * resistance * wheelRadius * wheelRadius * mass) * velocity + gearRatio * Kt / (resistance * wheelRadius * mass) * voltage;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public double getPosition() {
		return position;
	}
	
	public double getVoltage() {
		return voltage;
	}
	

}
