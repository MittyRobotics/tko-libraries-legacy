package com.amhsrobotics.libs.auton.motionprofile;

import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.math.Function;

public class JointVelocityMotionProfile {
	
	private final MotionSegment[] motionSegments;
	
	private final MotionSegment totalMotionSegment;
	
	private final VelocityConstraints velocityConstraints;
	
	private final double totalTime;
	
	public JointVelocityMotionProfile(JointVelocityMotionProfile.MotionSegment[] motionSegments, VelocityConstraints velocityConstraints){
		this.motionSegments = motionSegments;
		this.velocityConstraints = velocityConstraints;
		totalTime = motionSegments[motionSegments.length-1].getEndTime();
		this.totalMotionSegment = new TrapezoidalMotionSegment(0,totalTime,velocityConstraints);
	}
	
	public double getVelocityAtTime(double t){
		for(int i = 0; i < motionSegments.length; i++){
			if(t > motionSegments[i].getStartTime() && t < motionSegments[i].getEndTime()){
				return Math.min(motionSegments[i].getVelocityAtTime(t),totalMotionSegment.getVelocityAtTime(t));
			}
		}
		return 0;
	}
	
	public MotionSegment[] getMotionSegments() {
		return motionSegments;
	}
	
	public double getTotalTime() {
		return totalTime;
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	public MotionSegment getTotalMotionSegment() {
		return totalMotionSegment;
	}
	
	public static abstract class MotionSegment {
		
		
		public abstract double getVelocityAtTime(double t);
		public abstract double getStartTime();
		public abstract double getEndTime();
		
		
	}
	
	public static class FlatMotionSegment extends MotionSegment{
		
		private final double velocity;
		private final double startTime;
		private final double endTime;
		
		public FlatMotionSegment(double velocity, double startTime, double endTime) {
			
			this.velocity = velocity;
			this.startTime = startTime;
			this.endTime = endTime;
		}
		
		@Override
		public double getVelocityAtTime(double t) {
			return velocity;
		}
		
		@Override
		public double getStartTime() {
			return startTime;
		}
		
		@Override
		public double getEndTime() {
			return endTime;
		}
	}
	
	public static class TrapezoidalMotionSegment extends MotionSegment{
		
		private double startTime;
		private double endTime;
		private double startVelocity;
		private double endVelocity;
		private VelocityConstraints velocityConstraints;
		private TrapezoidalMotionProfile motionProfile;
		
		public TrapezoidalMotionSegment(double startTime, double endTime, VelocityConstraints velocityConstraints) {
			this.startTime = startTime;
			this.endTime = endTime;
			this.startVelocity = startVelocity;
			this.endVelocity = endVelocity;
			this.velocityConstraints = velocityConstraints;
			this.motionProfile = new TrapezoidalMotionProfile(endTime-startTime,velocityConstraints);
		}
		
		public double getVelocityAtTime(double t){
			return motionProfile.getVelocityAtTime(t-startTime);
		}
		
		public double getStartTime() {
			return startTime;
		}
		
		public void setStartTime(double startTime) {
			this.startTime = startTime;
		}
		
		public double getEndTime() {
			return endTime;
		}
		
		public void setEndTime(double endTime) {
			this.endTime = endTime;
		}
		
		public double getStartVelocity() {
			return startVelocity;
		}
		
		public void setStartVelocity(double startVelocity) {
			this.startVelocity = startVelocity;
		}
		
		public double getEndVelocity() {
			return endVelocity;
		}
		
		public void setEndVelocity(double endVelocity) {
			this.endVelocity = endVelocity;
		}
		
		public VelocityConstraints getVelocityConstraints() {
			return velocityConstraints;
		}
	}
	
}
