package com.amhsrobotics.libs.auton.path.follow;

import com.amhsrobotics.libs.auton.motionprofile.LimitVelocityMotion;
import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.datatypes.DrivetrainWheelVelocities;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.math.geometry.Transform;

import javax.swing.*;

public class AutonDriver {
	private static AutonDriver instance = new AutonDriver();
	
	public static AutonDriver getInstance(){
		return instance;
	}
	
	/** Default lookahead distance for the {@link PurePursuitController}. This is the distance of the point a head of the robot that it targets. */
	public static double PATH_DEFAULT_LOOKAHEAD = 20;
	/** Adaptive lookahead distance scale factor. This number is multiplied by the tracking error (distance between current point and expected point) of the {@link PurePursuitController} to get an adapted lookahead distance. */
	public static double PATH_ADAPTIVE_LOOKAHEAD_SCALE = 2;
	/** The distance between the left and right set of wheels, also known as the track width of the chassis */
	public static double TRACK_WIDTH = 27;
	/**
	 * The robot's transform object, containing the robot's position and heading. Update this with your odometry values
	 * periodically so the path followers know where the robot is.
	 */
	private Transform robotTransform;
	
	/** The robot's current velocity. Update this periodically with your robot's wheel velocities */
	private DrivetrainWheelVelocities robotWheelVelocities;
	
	public static VelocityConstraints PATH_FOLLOWING_VELOCITY_CONSTRAINTS = new VelocityConstraints(20,20,100,10,0,0);
	
	private Path currentPath;
	
	private PurePursuitController currentPurePursuitController;
	
	private double time;
	
	private AutonDriver(){
	
	}
	
	public DrivetrainWheelVelocities updatePurePursuit(double timestamp){
		double deltaTime = timestamp-time;
		this.time = timestamp;
		
		PurePursuitController.Output purePursuitOutput = currentPurePursuitController.update();
		
		double adjustedLinearVelocity = LimitVelocityMotion.getInstance().limitVelocity(robotWheelVelocities.getAvgVelocity(),purePursuitOutput.desiredLinearVelocity,deltaTime,PATH_FOLLOWING_VELOCITY_CONSTRAINTS);
		
		return getWheelVelocitiesFromRadius(adjustedLinearVelocity,purePursuitOutput.turningRadius);
	}
	
	public DrivetrainWheelVelocities getWheelVelocitiesFromRadius(double baseVelocity, double arcRadius){
		double angularVelocity = baseVelocity / arcRadius;
		
		return new DrivetrainWheelVelocities( angularVelocity * (arcRadius - (TRACK_WIDTH / 2)),angularVelocity * (arcRadius + (TRACK_WIDTH / 2)));
	}
	
	/**
	 * Updates the robot's position and wheel velocities.
	 *
	 * Update this every time you update your path follower so the path follower knows where the robot is.
	 *
	 * @param robotTransform the robot's {@link Transform} object holding its position and rotation is 2d space
	 * @param wheelVelociteis the robot's left and right wheel velocity
	 */
	public void updateRobotState(Transform robotTransform, DrivetrainWheelVelocities wheelVelociteis) {
		this.robotTransform = robotTransform;
		this.robotWheelVelocities = wheelVelociteis;
	}
	
	public Transform getRobotTransform() {
		return robotTransform;
	}
	
	public DrivetrainWheelVelocities getRobotWheelVelocities(){
		return robotWheelVelocities;
	}
	
	public double getTime() {
		return time;
	}
	
	public void setTime(double time) {
		this.time = time;
	}
	
}
