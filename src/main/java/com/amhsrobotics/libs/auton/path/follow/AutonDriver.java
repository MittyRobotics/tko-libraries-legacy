package com.amhsrobotics.libs.auton.path.follow;

import com.amhsrobotics.libs.datatypes.DrivetrainWheelVelocities;
import com.amhsrobotics.libs.math.geometry.Transform;

public class AutonDriver {
	private static AutonDriver instance = new AutonDriver();
	
	public static AutonDriver getInstance(){
		return instance;
	}
	
	/** Default lookahead distance for the {@link PurePursuitController}. This is the distance of the point a head of the robot that it targets. */
	private static double PATH_DEFAULT_LOOKAHEAD = 20;
	/** Adaptive lookahead distance scale factor. This number is multiplied by the tracking error (distance between current point and expected point) of the {@link PurePursuitController} to get an adapted lookahead distance. */
	private static double PATH_ADAPTIVE_LOOKAHEAD_SCALE = 2;
	/** The distance between the left and right set of wheels, also known as the track width of the chassis */
	private static double TRACK_WIDTH = 27;
	
	
	private Transform robotTransform;
	
	private AutonDriver(){
	
	}
	
	public DrivetrainWheelVelocities getWheelVelocitiesFromRadius(double baseVelocity, double arcRadius){
		double angularVelocity = baseVelocity / arcRadius;
		
		return new DrivetrainWheelVelocities( angularVelocity * (arcRadius - (AutonDriver.getInstance().getTrackWidth() / 2)),angularVelocity * (arcRadius + (AutonDriver.getInstance().getTrackWidth() / 2)));
	}
	
	/**
	 * Updates the robot's position with a new {@link Transform} object that represents its position in 2d space.
	 *
	 * Set this periodically to the current robot's position using your odometry calculations.
	 *
	 * @param robotTransform
	 */
	public void updateRobotTransform(Transform robotTransform) {
		this.robotTransform = robotTransform;
	}
	
	public Transform getRobotTransform() {
		return robotTransform;
	}
	public double getTrackWidth() {
		return TRACK_WIDTH;
	}
	
	

}
