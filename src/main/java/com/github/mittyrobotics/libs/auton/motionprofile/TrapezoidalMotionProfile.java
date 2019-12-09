package com.github.mittyrobotics.libs.auton.motionprofile;

import com.github.mittyrobotics.libs.util.Function;
import com.github.mittyrobotics.libs.datatypes.MechanismBounds;
import com.github.mittyrobotics.libs.datatypes.MotionFrame;
import com.github.mittyrobotics.libs.datatypes.VelocityConstraints;
import com.github.mittyrobotics.libs.util.IntegralMath;

public class TrapezoidalMotionProfile {
	private TrapezoidalMotionProfile.MotionSegment accelerationSegment;
	private TrapezoidalMotionProfile.MotionSegment cruiseSegment;
	private TrapezoidalMotionProfile.MotionSegment decelerationSegment;
	private double tTotal;
	
	private double setpoint;
	private double adjustedSetpoint;
	private double maxAcceleration;
	private double maxDeceleration;
	private double startVelocity;
	private double endVelocity;
	private double maxVelocity;
	private double startPosition;
	
	private double minPosition;
	private double maxPosition;
	
	private double prevTime;
	private double prevVelocity;
	
	private boolean isFinished;
	
	/**
	 * TrapezoidalMotionProfile Constructor
	 * <p>
	 * Creates a new trapezoidal motion profile
	 * <p>
	 * This motion profile works in both absolute and relative space. Absolute space is where a setpoint of 2000 ticks
	 * will move the motor to the encoder position of 2000, and relative space is where a setpoint of 2000 ticks will
	 * move the motor 2000 ticks further than it's current position.
	 * <p>
	 * To use absolute space, input the mechanisms's starting position in the MechanismBounds parameter. To use relative
	 * space, leave the starting position at 0.
	 * <p>
	 * If the mechanism does not have any minimum and maximum position bounds (ex: flywheel, drive motor), leave the
	 * minimum and maximum bounds in the MechanismBounds parameter at 0.
	 *
	 * @param setpoint            setpoint
	 * @param velocityConstraints velocity constraints of the motion profile
	 * @param mechanismBounds     the min and max position bounds as well as starting position
	 */
	public TrapezoidalMotionProfile(double setpoint, VelocityConstraints velocityConstraints, MechanismBounds mechanismBounds) {
		if (setpoint < 0) {
			velocityConstraints.setMaxAcceleration(-velocityConstraints.getMaxAcceleration());
			velocityConstraints.setMaxVelocity(-velocityConstraints.getMaxVelocity());
			velocityConstraints.setMaxDeceleration(-velocityConstraints.getMaxDeceleration());
		}
		
		this.maxAcceleration = velocityConstraints.getMaxAcceleration();
		this.maxDeceleration = velocityConstraints.getMaxDeceleration();
		this.startVelocity = velocityConstraints.getStartVelocity();
		this.endVelocity = velocityConstraints.getEndVelocity();
		this.maxVelocity = velocityConstraints.getMaxVelocity();
		this.startPosition = mechanismBounds.getCurrentPosition();
		this.minPosition = mechanismBounds.getMinPosition();
		this.maxPosition = mechanismBounds.getMaxPosition();
		this.setpoint = setpoint;
		this.adjustedSetpoint = setpoint - mechanismBounds.getCurrentPosition();
		
		//Initial calculation
		calculateMotionProfile();
		
		
		double c = accelerationSegment.getTime();
		
		double f = cruiseSegment.getTime() + accelerationSegment.getTime();
		
		double finalVelocity = IntegralMath.integral(f, tTotal, decelerationSegment.getF()) + IntegralMath.integral(0, c, accelerationSegment.getF()) + IntegralMath.integral(c, f, cruiseSegment.getF());
		
		double setpointDifference = adjustedSetpoint - finalVelocity;
		
		this.adjustedSetpoint = adjustedSetpoint + setpointDifference;
		
		//Adjusted setpoint calculation
		calculateMotionProfile();
	}
	
	/**
	 * Generates a TrapezoidalMotionProfile based on time instead of setpoint. This is primarily used for motion
	 * profiles that want to run at a velocity for a certain time.
	 *
	 * @param time
	 * @param velocityConstraints
	 */
	public TrapezoidalMotionProfile(double time, VelocityConstraints velocityConstraints) {
		this.maxAcceleration = velocityConstraints.getMaxAcceleration();
		this.maxDeceleration = velocityConstraints.getMaxDeceleration();
		this.startVelocity = velocityConstraints.getStartVelocity();
		this.endVelocity = velocityConstraints.getEndVelocity();
		this.maxVelocity = velocityConstraints.getMaxVelocity();
		
		double tTotal = time;
		double tAccel = (maxVelocity - startVelocity) / maxAcceleration;
		double tDecel = (maxVelocity - endVelocity) / maxDeceleration;
		double tCruise = tTotal - tAccel - tDecel;
		if (tCruise < 0) {
			double overTime = Math.abs(tCruise);
			double accelTimeRatio = tAccel / (tTotal + overTime);
			double decelTimeRatio = tDecel / (tTotal + overTime);
			tAccel -= overTime * accelTimeRatio;
			tDecel -= overTime * decelTimeRatio;
			tCruise = 0;
			tTotal = tAccel + tDecel;
			this.maxVelocity = tAccel * maxAcceleration + startVelocity;
		}
		this.tTotal = tTotal;
		
		accelerationSegment = new TrapezoidalMotionProfile.MotionSegment(tAccel);
		cruiseSegment = new TrapezoidalMotionProfile.MotionSegment(tCruise);
		decelerationSegment = new TrapezoidalMotionProfile.MotionSegment(tDecel);
	}
	
	/**
	 * Calculates the outline (the 3 {@link TrapezoidalMotionProfile.MotionSegment}s) of the motion profile.
	 */
	private void calculateMotionProfile() {
		
		double zeroToStartVelocityTime = startVelocity / maxAcceleration;
		double zeroToStartVelocityDistance = zeroToStartVelocityTime * zeroToStartVelocityTime * maxAcceleration / 2;
		
		double endVelocityToZeroTime = endVelocity / maxDeceleration;
		double endVelocityToZeroDistance = endVelocityToZeroTime * endVelocityToZeroTime * maxDeceleration / 2;
		
		double totalDistanceWithEnds = zeroToStartVelocityDistance + adjustedSetpoint + endVelocityToZeroDistance;
		
		double triangleDDecel;
		double triangleDAccel;
		
		if (maxAcceleration >= maxDeceleration) {
			double accelDecelRatio = (maxAcceleration / (maxDeceleration + maxAcceleration));
			
			triangleDAccel = totalDistanceWithEnds * accelDecelRatio;
			triangleDDecel = totalDistanceWithEnds - triangleDAccel;
		} else {
			double accelDecelRatio = (maxDeceleration / (maxAcceleration + maxDeceleration));
			
			triangleDDecel = totalDistanceWithEnds * accelDecelRatio;
			triangleDAccel = totalDistanceWithEnds - triangleDDecel;
		}
		
		double theoreticalMaxVelocity = Math.sqrt(2 * maxAcceleration * triangleDAccel);
		
		double tAccel, tDecel, dAccel, dDecel, dCruise, tCruise, tTotal;
		if (startVelocity >= theoreticalMaxVelocity) {
			theoreticalMaxVelocity = startVelocity;
			maxVelocity = Math.min(theoreticalMaxVelocity, maxVelocity);
			tAccel = 0;
			dAccel = 0;
			tDecel = maxVelocity / maxDeceleration;
			dDecel = (maxDeceleration * tDecel * tDecel) / 2;
			dCruise = totalDistanceWithEnds - dAccel - dDecel;
			
			tCruise = dCruise / maxVelocity;
			
			if (dCruise < 0.01) {
				dCruise = 0;
				tCruise = 0;
			}
			tAccel = tAccel - zeroToStartVelocityTime;
			tDecel = tDecel - endVelocityToZeroTime;
			
			tTotal = tAccel + tCruise + tDecel;
			
			this.tTotal = tTotal;
		} else if (endVelocity >= theoreticalMaxVelocity) {
			theoreticalMaxVelocity = endVelocity;
			maxVelocity = Math.min(theoreticalMaxVelocity, maxVelocity);
			tDecel = 0;
			dDecel = 0;
			tAccel = maxVelocity / maxAcceleration;
			dAccel = (maxAcceleration * tAccel * tAccel) / 2;
			dCruise = totalDistanceWithEnds - dAccel - dDecel;
			
			tCruise = dCruise / maxVelocity;
			
			if (dCruise < 0.01) {
				dCruise = 0;
				tCruise = 0;
			}
			tAccel = tAccel - zeroToStartVelocityTime;
			tDecel = tDecel - endVelocityToZeroTime;
			
			tTotal = tAccel + tCruise + tDecel;
			
			this.tTotal = tTotal;
		} else {
			maxVelocity = Math.min(theoreticalMaxVelocity, maxVelocity);
			
			tAccel = maxVelocity / maxAcceleration;
			
			tDecel = maxVelocity / maxDeceleration;
			
			dAccel = (maxAcceleration * tAccel * tAccel) / 2;
			
			dDecel = (maxDeceleration * tDecel * tDecel) / 2;
			
			dCruise = totalDistanceWithEnds - dAccel - dDecel;
			
			tCruise = dCruise / maxVelocity;
			
			if (dCruise < 0.01) {
				dCruise = 0;
				tCruise = 0;
			}
			
			tAccel = tAccel - zeroToStartVelocityTime;
			tDecel = tDecel - endVelocityToZeroTime;
			
			tTotal = tAccel + tCruise + tDecel;
			
			this.tTotal = tTotal;
			
			if (tAccel < 0.01) {
				tAccel = 0;
				dAccel = 0;
			}
			
			if (tDecel < 0.01) {
				tDecel = 0;
				dDecel = 0;
			}
		}
		
		
		System.out.println(maxVelocity + " " + tAccel + " " + tCruise + " " + tDecel + " " + dAccel + " " + dCruise + " " + dDecel + " " + (dAccel + dCruise + dDecel));
		
		double d = maxVelocity;
		double a = startVelocity;
		double c = tAccel;
		double h = endVelocity;
		double g = tTotal;
		double f = tCruise + tAccel;
		
		Function accelerationFunction = new Function() {
			@Override
			public double f(double x) {
				return ((d - a) / c) * x + a;
			}
		};
		Function cruiseFunction = new Function() {
			@Override
			public double f(double x) {
				return d;
			}
		};
		Function decelerationFunction = new Function() {
			@Override
			public double f(double x) {
				return (h - d) / (g - f) * (x - f) + d;
			}
		};
		
		accelerationSegment = new TrapezoidalMotionProfile.MotionSegment(tAccel, dAccel, accelerationFunction);
		cruiseSegment = new TrapezoidalMotionProfile.MotionSegment(tCruise, dCruise, cruiseFunction);
		decelerationSegment = new TrapezoidalMotionProfile.MotionSegment(tDecel, dDecel, decelerationFunction);
		
		if (maxAcceleration == 0 || maxDeceleration == 0) {
			accelerationSegment = new TrapezoidalMotionProfile.MotionSegment(0, 0, accelerationFunction);
			cruiseSegment = new TrapezoidalMotionProfile.MotionSegment(0, 0, cruiseFunction);
			decelerationSegment = new TrapezoidalMotionProfile.MotionSegment(0, 0, decelerationFunction);
		}
	}
	
	/**
	 * Calculates the {@link MotionFrame} at a certain time.
	 *
	 * @param t time of the motion frame
	 * @return a new {@link MotionFrame} at time t
	 */
	public MotionFrame getFrameAtTime(double t) {
		
		double velocity = getVelocityAtTime(t);
		double position = getPositionAtTime(t);
		double acceleration = getAccelerationAtTime(t, velocity);
		
		if (!(minPosition == 0 && maxPosition == 0)) {
			position = Math.min(position, maxPosition);
			position = Math.max(position, minPosition);
		}
		
		isFinished = t <= tTotal;
		
		if (t < tTotal) {
			return new MotionFrame(position, velocity, acceleration, t);
		} else {
			return new MotionFrame(setpoint, endVelocity, acceleration, t);
		}
	}
	
	/**
	 * Returns the velocity of the motion profile at time t.
	 *
	 * @param t the time of the desired velocity value
	 * @return the velocity of the motion profile at time t
	 */
	public double getVelocityAtTime(double t) {
		double output;
		if (t < accelerationSegment.getTime()) {
			output = t * maxAcceleration + startVelocity;
			
		} else if (t < cruiseSegment.getTime() + accelerationSegment.getTime()) {
			output = maxVelocity;
		} else if (t >= tTotal) {
			output = endVelocity;
		} else {
			output = maxVelocity - (t - accelerationSegment.getTime() - cruiseSegment.getTime()) * maxDeceleration;
		}
		
		return output;
	}
	
	/**
	 * Returns the position of the motion profile at time t.
	 *
	 * @param t the time of the desired position value
	 * @return the position of the motion profile at time t
	 */
	private double getPositionAtTime(double t) {
		double output = 0;
		
		double c = accelerationSegment.getTime();
		double f = cruiseSegment.getTime() + accelerationSegment.getTime();
		
		if (t <= accelerationSegment.getTime()) {
			output = IntegralMath.integral(0, t, accelerationSegment.getF());
		} else if (t <= cruiseSegment.getTime() + accelerationSegment.getTime()) {
			output = IntegralMath.integral(c, t, cruiseSegment.getF()) + IntegralMath.integral(0, c, accelerationSegment.getF());
		} else {
			output = IntegralMath.integral(f, t, decelerationSegment.getF()) + IntegralMath.integral(0, c, accelerationSegment.getF()) + IntegralMath.integral(c, f, cruiseSegment.getF());
		}
		
		return output + startPosition;
	}
	
	/**
	 * Only works with starting and ending velocity of 0.
	 *
	 * @param position
	 * @return
	 */
	public double getTimeAtPosition(double position) {
		if (position < accelerationSegment.distance) {
			return Math.sqrt((2 * position) / maxAcceleration);
		} else if (position > accelerationSegment.distance && position < cruiseSegment.distance) {
			return accelerationSegment.getTime() +
					(position - accelerationSegment.getDistance()) / maxVelocity;
		} else {
			return accelerationSegment.getTime() +
					cruiseSegment.getTime() +
					Math.sqrt((2 * (position - accelerationSegment.getDistance() - cruiseSegment.getDistance())) / maxAcceleration);
		}
	}
	
	/**
	 * Returns the acceleration of the motion profile at time t.
	 *
	 * @param t the time of the desired acceleration value
	 * @return the acceleration of the motion profile at time t
	 */
	private double getAccelerationAtTime(double t, double velocity) {
		
		double acceleration;
		if (t == 0) {
			acceleration = 0;
		} else {
			acceleration = (velocity - prevVelocity) / (t - prevTime);
		}
		
		this.prevVelocity = velocity;
		this.prevTime = t;
		return acceleration;
	}
	
	public double getSetpoint() {
		return setpoint;
	}
	
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}
	
	public double getMaxAcceleration() {
		return maxAcceleration;
	}
	
	public void setMaxAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
	}
	
	public double getMaxDeceleration() {
		return maxDeceleration;
	}
	
	public void setMaxDeceleration(double maxDeceleration) {
		this.maxDeceleration = maxDeceleration;
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
	
	public double getMaxVelocity() {
		return maxVelocity;
	}
	
	public void setMaxVelocity(double maxVelocity) {
		this.maxVelocity = maxVelocity;
	}
	
	public double getStartPosition() {
		return startPosition;
	}
	
	public void setStartPosition(double startPosition) {
		this.startPosition = startPosition;
	}
	
	public TrapezoidalMotionProfile.MotionSegment getAccelerationSegment() {
		return accelerationSegment;
	}
	
	public void setAccelerationSegment(TrapezoidalMotionProfile.MotionSegment accelerationSegment) {
		this.accelerationSegment = accelerationSegment;
	}
	
	public TrapezoidalMotionProfile.MotionSegment getCruiseSegment() {
		return cruiseSegment;
	}
	
	public void setCruiseSegment(TrapezoidalMotionProfile.MotionSegment cruiseSegment) {
		this.cruiseSegment = cruiseSegment;
	}
	
	public TrapezoidalMotionProfile.MotionSegment getDecelerationSegment() {
		return decelerationSegment;
	}
	
	public void setDecelerationSegment(TrapezoidalMotionProfile.MotionSegment decelerationSegment) {
		this.decelerationSegment = decelerationSegment;
	}
	
	public double gettTotal() {
		return tTotal;
	}
	
	public void settTotal(double tTotal) {
		this.tTotal = tTotal;
	}
	
	public double getMinPosition() {
		return minPosition;
	}
	
	public void setMinPosition(double minPosition) {
		this.minPosition = minPosition;
	}
	
	public double getMaxPosition() {
		return maxPosition;
	}
	
	public void setMaxPosition(double maxPosition) {
		this.maxPosition = maxPosition;
	}
	
	public double getPrevTime() {
		return prevTime;
	}
	
	public void setPrevTime(double prevTime) {
		this.prevTime = prevTime;
	}
	
	public double getPrevVelocity() {
		return prevVelocity;
	}
	
	public void setPrevVelocity(double prevVelocity) {
		this.prevVelocity = prevVelocity;
	}
	
	/**
	 * Returns if the time inputted into the motion profile is greater than or equal to the calculated total time of the
	 * motion profile.
	 * <p>
	 * This returns if the calculated motion profile is finished, however the actual mechanism will not have exactly
	 * the same motion as the motion profile, so it is not a good practice to use this as your final check to see if the
	 * motion is finished.
	 *
	 * @return if the motion profile is finished
	 */
	public boolean isFinished() {
		return isFinished;
	}
	
	public void setFinished(boolean finished) {
		isFinished = finished;
	}
	
	
	public class MotionSegment {
		private double t;
		private double distance;
		private Function f;
		
		public MotionSegment(double t, double distance, Function f) {
			this.t = t;
			this.distance = distance;
			this.f = f;
		}
		
		public MotionSegment(double t) {
			this.t = t;
		}
		
		public double getTime() {
			return t;
		}
		
		public void setTime(double t) {
			this.t = t;
		}
		
		public double getDistance() {
			return distance;
		}
		
		public Function getF() {
			return f;
		}
		
		public void setF(Function f) {
			this.f = f;
		}
	}
}

