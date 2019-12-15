package com.github.mittyrobotics.path.following.datatypes;

import com.github.mittyrobotics.datatypes.motion.DrivetrainVelocities;

/**
 * Contains differential drive kinematics equations for figuring out wheel velocities.
 *
 * http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 *
 */
public class DifferentialDriveKinematics {
	private static DifferentialDriveKinematics instance = new DifferentialDriveKinematics();
	
	private double trackWidth;
	
	public static DifferentialDriveKinematics getInstance() {
		return instance;
	}
	
	private DifferentialDriveKinematics(){
	
	}
	
	/**
	 * Calculates the {@link DrivetrainVelocities} given a base robot velocity and a radius of the circle that it wants
	 * to follow using differential drive kinematics.
	 *
	 * @param linearVelocity the base linear robot velocity in units per second (how fast the robot moves forward).
	 * @param radius the radius to follow in units.
	 * @return the calculated {@link DrivetrainVelocities}.
	 */
	public DrivetrainVelocities calculateFromRadius(double linearVelocity, double radius){
		//Calculate the angular velocity of the robot in radians per second
		double angularVelocity = linearVelocity / radius;
		
		//If track width has not been set, send a warning and return velocities of 0 to avoid any damage
		if(trackWidth == 0){
			System.out.println("WARNING: Track width in DifferentialDriveKinematics.java has not been set!");
			return new DrivetrainVelocities(0,0);
		}
		
		//Return the calculated drivetrain velocities
		return new DrivetrainVelocities(angularVelocity * (radius - (trackWidth / 2)), angularVelocity * (radius + (trackWidth / 2)));
	}
	
	/**
	 * Calculates the {@link DrivetrainVelocities} given a base robot velocity and a radius of the circle that it wants
	 * to follow using differential drive kinematics.
	 *
	 * @param linearVelocity the base linear robot velocity in units per second (how fast the robot moves forward).
	 * @param angularVelocity the angular velocity of the robot in radians per second
	 * @return the calculated {@link DrivetrainVelocities}.
	 */
	public DrivetrainVelocities calculateFromAngularVelocity(double linearVelocity, double angularVelocity){
		//Calculate the radius given linear velocity and angular velocity
		double radius = linearVelocity/angularVelocity;
		
		//If track width has not been set, send a warning and return velocities of 0 to avoid any damage
		if(trackWidth == 0){
			System.out.println("WARNING: Track width in DifferentialDriveKinematics.java has not been set!");
			return new DrivetrainVelocities(0,0);
		}
		
		//Return the calculated drivetrain velocities
		return new DrivetrainVelocities(angularVelocity * (radius - (trackWidth / 2)), angularVelocity * (radius + (trackWidth / 2)));
	}
	
	public double getTrackWidth(){
		return trackWidth;
	}
	
	public void setTrackWidth(double trackWidth) {
		this.trackWidth = trackWidth;
	}
}