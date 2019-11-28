package com.amhsrobotics.libs.auton.path.follow;

import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.auton.path.generation.TrajectoryPoint;
import com.amhsrobotics.libs.datatypes.DrivetrainWheelVelocities;
import com.amhsrobotics.libs.math.geometry.Arc;
import com.amhsrobotics.libs.math.geometry.Position;
import com.amhsrobotics.libs.math.geometry.Rotation;

public class PurePursuitController {
	
	private final Path path;
	
	private double t;
	
	private TrajectoryPoint currentClosestPoint;
	private TrajectoryPoint currentTargetPoint;
	
	private int prevTargetIndex;
	
	public PurePursuitController(Path path){
		this.path = path;
	}
	
	/**
	 * Initializes the {@link PurePursuitController}. Returns the timestamp that the robot starts on.
	 *
	 * It returns the timestamp in case the robot starts somewhere off the start of the path, the robot will be able to
	 * follow the path's {@link com.amhsrobotics.libs.auton.motionprofile.JointVelocityMotionProfile} velocity motion
	 * profile from its current position.
	 *
	 * @return the timestamp along the path that the robot starts on in seconds.
	 */
	public double initAndRecieveTimestamp(){
		return getClosestPoint(0).getTime();
	}
	
	public DrivetrainWheelVelocities advance(double deltaTime, double lookaheadDistance){
		t += deltaTime;
		
		currentClosestPoint = getClosestPoint(0);
		currentTargetPoint = getClosestPoint(lookaheadDistance);
		
		Arc arcToTargetPoint = AutonDriver.getInstance().getRobotTransform().findTangentIntersectionArc(currentTargetPoint);
		
		return AutonDriver.getInstance().getWheelVelocitiesFromRadius(0,arcToTargetPoint.getRadius());
	}
	
	/**
	 * Finds the closest point to the robot that is distanceShift away from the robot.
	 *
	 * To get the closest point to the robot's actual position, set distanceShift to 0. To get the closest point that is
	 * a certain distance away from the robot, such as the lookahead distance, set distanceShift to that distance.
	 *
	 * The point returned will always be the closest point in front of the robot
	 *
	 * @param distanceShift the distance away from the robot to find the closest point to
	 * @return
	 */
	public TrajectoryPoint getClosestPoint(double distanceShift){
		TrajectoryPoint point = null;
		if (path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].getPosition().distance(AutonDriver.getInstance().getRobotTransform().getPosition()) < distanceShift) {
			final double angle = path.getWaypoints()[path.getWaypoints().length - 1].getRotation().getHeading();
			final double newLookahead = distanceShift - path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].getPosition().distance(AutonDriver.getInstance().getRobotTransform().getPosition());
			final double x = Math.cos(Math.toRadians(angle)) * newLookahead + path.getWaypoints()[path.getWaypoints().length - 1].getPosition().getX();
			final double y = Math.sin(Math.toRadians(angle)) * newLookahead + path.getWaypoints()[path.getWaypoints().length - 1].getPosition().getY();
			prevTargetIndex = path.getTrajectoryPoints().length - 1;
			point = new TrajectoryPoint(new Position(x,y), new Rotation());
		} else {
			double currentClosest = 9999;
			for (int i = prevTargetIndex; i < path.getTrajectoryPoints().length; i++) {
				if (Math.abs(path.getTrajectoryPoints()[i].getPosition().distance(AutonDriver.getInstance().getRobotTransform().getPosition())) - distanceShift < currentClosest) {
					currentClosest = Math.abs(path.getTrajectoryPoints()[i].getPosition().distance(AutonDriver.getInstance().getRobotTransform().getPosition())) - distanceShift;
					prevTargetIndex = i;
					point = path.getTrajectoryPoints()[i];
				}
			}
		}
		return point;
	}
	
	public Path getPath() {
		return path;
	}
	
	public double getTime() {
		return t;
	}
	
	public void setTime(double t) {
		this.t = t;
	}
	
	public TrajectoryPoint getCurrentClosestPoint() {
		return currentClosestPoint;
	}
	
	public void setCurrentClosestPoint(TrajectoryPoint currentClosestPoint) {
		this.currentClosestPoint = currentClosestPoint;
	}
	
	public TrajectoryPoint getCurrentTargetPoint() {
		return currentTargetPoint;
	}
	
	public void setCurrentTargetPoint(TrajectoryPoint currentTargetPoint) {
		this.currentTargetPoint = currentTargetPoint;
	}
	
}
