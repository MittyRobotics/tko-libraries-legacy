package com.github.mittyrobotics.simulation;


import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.visualization.graphs.RobotGraph;

import javax.swing.*;

public class RobotSimManager implements Runnable{
	

	private double periodTime;
	
	private double mass;
	private double driveGearRatio;
	private double driveWheelRadius;
	private double robotWidth;
	private double robotLength;
	
	private SimRobot robot;
	private SimDrivetrain drivetrain;
	
	private RobotGraph robotGraph = new RobotGraph();

	private boolean calledInit = false;
	
	private static RobotSimManager instance = new RobotSimManager();
	
	
	public static RobotSimManager getInstance() {
		return instance;
	}
	
	private RobotSimManager(){

	}
	
	public void setupRobotSimManager(SimRobot robot, SimDrivetrain drivetrain, double robotMass, double driveGearRatio, double driveWheelRadius, double robotWidth, double robotLength, double periodTime){
		calledInit = false;
		setupDrivetrainProperties(robotMass,driveGearRatio,driveWheelRadius,robotWidth,robotLength);
		setupRobot(robot,drivetrain);
		setPeriodTime(periodTime);
		initHardware();
		init();
	}
	
	private void setupRobot(SimRobot robot, SimDrivetrain drivetrain){
		this.robot = robot;
		this.drivetrain = drivetrain;
	}
	
	private void setPeriodTime(double periodTime){
		this.periodTime = periodTime;
	}
	
	private void setupDrivetrainProperties(double mass, double gearRatio, double wheelRadius, double width, double length){
		this.mass = mass;
		this.driveGearRatio = gearRatio;
		this.driveWheelRadius = wheelRadius;
		this.robotWidth = width;
		this.robotLength = length;
	}
	
	
	@Override
	public void run() {
		while(true){
			while(!calledInit) {
			}
			periodic();
			try {
				Thread.sleep((long) (periodTime * 1000));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	private void initHardware(){
		drivetrain.initDrivetrain();
		new Thread(this).start();
	}
	
	private void init(){
		robot.robotInit();
		calledInit = true;
	}
	
	/**
	 * Place all of the updated functions in this
	 */
	private void periodic(){
		robot.robotPeriodic();
		drivetrain.odometry();
		SwingUtilities.invokeLater(() -> {
			robotGraph.graphRobot(new Transform(drivetrain.getRobotX(), drivetrain.getRobotY(), drivetrain.getHeading()),robotWidth,robotLength);
		});
	}
	
	
	public double getPeriodTime() {
		return periodTime;
	}
	
	public double getMass() {
		return mass;
	}
	
	public double getDriveGearRatio() {
		return driveGearRatio;
	}
	
	public double getDriveWheelRadius() {
		return driveWheelRadius;
	}
	
	public double getRobotWidth() {
		return robotWidth;
	}
	
	public double getRobotLength() {
		return robotLength;
	}
}
