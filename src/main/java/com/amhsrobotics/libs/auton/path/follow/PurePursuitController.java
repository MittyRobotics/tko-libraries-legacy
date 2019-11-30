package com.amhsrobotics.libs.auton.path.follow;

import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.util.path.TrajectoryPoint;
import com.amhsrobotics.libs.util.geometry.Arc;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Rotation;
import com.amhsrobotics.libs.util.geometry.Transform;

public class PurePursuitController {
	
	public static class Output{
		
		public double desiredLinearVelocity;
		public double turningRadius;
		
		public Output(double desiredLinearVelocity, double turningRadius){
			
			this.desiredLinearVelocity = desiredLinearVelocity;
			this.turningRadius = turningRadius;
		}
	}
	
	private final Path path;
	
	private TrajectoryPoint roughClosestPoint;
	private TrajectoryPoint actualClosestPoint;
	private int currentClosestPointIndex;
	private TrajectoryPoint roughTargetPoint;
	private TrajectoryPoint actualTargetPoint;
	private int currentTargetPointIndex;
	
	private int previousSearchIndex;
	
	private boolean lookaheadPastLastPoint = false;
	
	public PurePursuitController(Path path){
		this.path = path;
	}
	
	public Output update(){
		roughClosestPoint = getClosestPoint(AutonDriver.getInstance().getRobotTransform(),0,currentClosestPointIndex, true);
		currentClosestPointIndex = previousSearchIndex;
		actualClosestPoint = findClosestPointInSegment();
		roughTargetPoint = getClosestPoint(actualClosestPoint, AutonDriver.PATH_DEFAULT_LOOKAHEAD,currentClosestPointIndex, true);
		currentTargetPointIndex = previousSearchIndex;
		actualTargetPoint = findTargetPointInSegment();
		
		Arc arcToTargetPoint = AutonDriver.getInstance().getRobotTransform().findTangentIntersectionArc(roughTargetPoint);
		
		double desiredLinearVelocity = 0;
		
		return new Output(desiredLinearVelocity,arcToTargetPoint.getRadius());
	}
	
	/**
	 * Finds the closest point to the originPoint that is distanceShift away from the robot.
	 *
	 * To get the closest point to the robot's actual position, set distanceShift to 0 and originPoint to the robot's {@link Transform}. To get the closest point that is
	 * a certain distance away from the robot, such as the lookahead distance, set distanceShift to that distance.
	 *
	 * The point returned will always be the closest point in front of the robot
	 *
	 * @param originPoint the point to find the closest point to
	 * @param distanceShift the distance away from the robot to find the closest point to
	 * @return
	 */
	public TrajectoryPoint getClosestPoint(Transform originPoint, double distanceShift, int startSearchIndex, boolean roundUp){
		TrajectoryPoint point = null;
		if (path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].getPosition().distance(originPoint.getPosition()) < distanceShift) {
			final double angle = path.getWaypoints()[path.getWaypoints().length - 1].getRotation().getHeading();
			final double newLookahead = distanceShift - path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].getPosition().distance(originPoint.getPosition());
			final double x = Math.cos(Math.toRadians(angle)) * newLookahead + path.getWaypoints()[path.getWaypoints().length - 1].getPosition().getX();
			final double y = Math.sin(Math.toRadians(angle)) * newLookahead + path.getWaypoints()[path.getWaypoints().length - 1].getPosition().getY();
			previousSearchIndex = path.getTrajectoryPoints().length - 1;
			point = new TrajectoryPoint(new Position(x,y), new Rotation());
			lookaheadPastLastPoint = true;

		} else {
			double currentClosest = 9999;
			for (int i = startSearchIndex; i < path.getTrajectoryPoints().length; i++) {
				double distance = path.getTrajectoryPoints()[i].getPosition().distance(originPoint.getPosition()) - distanceShift;
				double distanceAbs = Math.abs(distance);
				if (distanceAbs < currentClosest) {
					boolean roundedUp = path.getTrajectoryPoints()[i].relativeTo(originPoint).getPosition().getX() > 0;
					if(roundUp && distance > 0 && roundedUp){
						currentClosest = distanceAbs;
						previousSearchIndex = i;
						point = path.getTrajectoryPoints()[i];
					}
					else if(!roundUp){
						currentClosest = distanceAbs;
						previousSearchIndex = i;
						point = path.getTrajectoryPoints()[i];
					}
					else{
						point = path.getTrajectoryPoints()[path.getTrajectoryPoints().length-1];
						previousSearchIndex = path.getTrajectoryPoints().length-1;
					}
				}
			}
		}
		return point;
	}
	
	/**
	 * Finds the closest point to the robot on the line in between the closest two {@link TrajectoryPoint}s.
	 * @return a new {@link TrajectoryPoint} containing the position of the point in between the two closest points to the robot.
	 */
	public TrajectoryPoint findClosestPointInSegment(){
		int secondPointIndex = currentClosestPointIndex;
//		if(currentClosestPointIndex != 0 && currentClosestPointIndex != path.getTrajectoryPoints().length-1){
//			double distanceBehind = path.getTrajectoryPoints()[currentClosestPointIndex-1].getPosition().distance(AutonDriver.getInstance().getRobotTransform().getPosition());
//			double distanceFront = path.getTrajectoryPoints()[currentClosestPointIndex+1].getPosition().distance(AutonDriver.getInstance().getRobotTransform().getPosition());
//
//			if(distanceBehind < distanceFront){
//				secondPointIndex = currentClosestPointIndex -1;
//			}
//			else{
//				secondPointIndex = currentClosestPointIndex;
//			}
//		}
//		else if(currentClosestPointIndex == 0){
//			secondPointIndex = 0;
//		}
//		else{
//			secondPointIndex = path.getTrajectoryPoints().length-1;
//		}
		
		Transform intersectingPoint = new Transform(AutonDriver.getInstance().getRobotTransform().getPosition(), path.getTrajectoryPoints()[secondPointIndex-1].getRotation().rotateBy(90)).findLineIntersectionPoint(path.getTrajectoryPoints()[secondPointIndex-1]);
		TrajectoryPoint trajectoryPoint = new TrajectoryPoint(intersectingPoint.getPosition(),intersectingPoint.getRotation());
		trajectoryPoint.setDistanceAlongPath(roughClosestPoint.getDistanceAlongPath());
//		trajectoryPoint.setRadius(roughClosestPoint.getRadius());
		trajectoryPoint.setTime(roughClosestPoint.getTime());
//		trajectoryPoint.setVelocity(roughClosestPoint.getVelocity());
		return trajectoryPoint;
	}
	
	/**
	 * Finds the position of the target point within the closest segment to the rough target point.
	 *
	 * @return a new {@link TrajectoryPoint} containing the position of the point within the segment closest to the target point.
	 */
	public TrajectoryPoint findTargetPointInSegment(){
		if(lookaheadPastLastPoint){
			return roughTargetPoint;
		}
		double distanceOffset =  actualClosestPoint.getPosition().distance(roughTargetPoint.getPosition()) -AutonDriver.PATH_DEFAULT_LOOKAHEAD ;
		double segmentDistance = path.getTrajectoryPoints()[currentTargetPointIndex-1].getPosition().distance(path.getTrajectoryPoints()[currentTargetPointIndex].getPosition());
		distanceOffset = Math.abs(distanceOffset-segmentDistance);
		TrajectoryPoint adjustedRoughTargetPoint;
		adjustedRoughTargetPoint = path.getTrajectoryPoints()[currentTargetPointIndex-1];

		System.out.println(distanceOffset);
		
		Transform transform = adjustedRoughTargetPoint.findPointDistanceAlongLine(distanceOffset);
		TrajectoryPoint trajectoryPoint = new TrajectoryPoint(transform.getPosition(), transform.getRotation());
		trajectoryPoint.setDistanceAlongPath(roughTargetPoint.getDistanceAlongPath());
		//trajectoryPoint.setRadius(roughTargetPoint.getRadius());
		trajectoryPoint.setTime(roughTargetPoint.getTime());
		//trajectoryPoint.setVelocity(roughTargetPoint.getVelocity());
		return trajectoryPoint;
	}
	
	
	public Path getPath() {
		return path;
	}
	

	public TrajectoryPoint getRoughClosestPoint() {
		return roughClosestPoint;
	}
	
	public void setRoughClosestPoint(TrajectoryPoint roughClosestPoint) {
		this.roughClosestPoint = roughClosestPoint;
	}
	
	public TrajectoryPoint getActualClosestPoint() {
		return actualClosestPoint;
	}
	
	public void setActualClosestPoint(TrajectoryPoint actualClosestPoint) {
		this.actualClosestPoint = actualClosestPoint;
	}
	
	public int getCurrentClosestPointIndex() {
		return currentClosestPointIndex;
	}
	
	public void setCurrentClosestPointIndex(int currentClosestPointIndex) {
		this.currentClosestPointIndex = currentClosestPointIndex;
	}
	
	public TrajectoryPoint getRoughTargetPoint() {
		return roughTargetPoint;
	}
	
	public void setRoughTargetPoint(TrajectoryPoint roughTargetPoint) {
		this.roughTargetPoint = roughTargetPoint;
	}
	
	public TrajectoryPoint getActualTargetPoint() {
		return actualTargetPoint;
	}
	
	public void setActualTargetPoint(TrajectoryPoint actualTargetPoint) {
		this.actualTargetPoint = actualTargetPoint;
	}
	
	public int getCurrentTargetPointIndex() {
		return currentTargetPointIndex;
	}
	
	public void setCurrentTargetPointIndex(int currentTargetPointIndex) {
		this.currentTargetPointIndex = currentTargetPointIndex;
	}
	
	
}
