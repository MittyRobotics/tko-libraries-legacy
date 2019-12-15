package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.motionprofile.util.Function;
import com.github.mittyrobotics.motionprofile.util.IntegralMath;
import com.github.mittyrobotics.motionprofile.util.datatypes.MechanismBounds;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.motionprofile.util.datatypes.MotionSegment;

public class TrapezoidalMotionProfile {
	
	private final MotionState startMotionState;
	private final MotionState endMotionState;
	private final VelocityConstraints velocityConstraints;
	private final MechanismBounds mechanismBounds;
	
	private MotionSegment accelerationSegment;
	private MotionSegment cruiseSegment;
	private MotionSegment decelerationSegment;
	
	private double tTotal;
	
	private double startPosition, startVelocity, endPosition, endVelocity, maxAcceleration, maxDeceleration, maxVelocity, minPosition, maxPosition;
	
	private boolean reversed;
	private boolean isFinished;
	
	/**
	 * Constructs a new {@link TrapezoidalMotionProfile}.
	 *
	 * @param startMotionState    The starting {@link MotionState} (the current state of motion). Only uses the
	 *                            position and velocity values from the {@link MotionState}.
	 * @param endMotionState      The ending {@link MotionState} (the desired state of motion). Only uses the
	 *                            position and velocity values from the {@link MotionState}.
	 * @param velocityConstraints The {@link VelocityConstraints} of the mechanism.
	 */
	public TrapezoidalMotionProfile(MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints) {
		this(startMotionState, endMotionState, velocityConstraints, new MechanismBounds(0, 0));
	}
	
	/**
	 * Constructs a new {@link TrapezoidalMotionProfile}.
	 *
	 * @param startMotionState    The starting {@link MotionState} (the current state of motion). Only uses the
	 *                            position and velocity values from the {@link MotionState}.
	 * @param endMotionState      The ending {@link MotionState} (the desired state of motion). Only uses the
	 *                            position and velocity values from the {@link MotionState}.
	 * @param velocityConstraints The {@link VelocityConstraints} of the mechanism.
	 * @param mechanismBounds     The {@link MechanismBounds} of the mechanism
	 */
	public TrapezoidalMotionProfile(MotionState startMotionState, MotionState endMotionState, VelocityConstraints velocityConstraints, MechanismBounds mechanismBounds) {
		this.startMotionState = startMotionState;
		this.endMotionState = endMotionState;
		this.velocityConstraints = velocityConstraints;
		this.mechanismBounds = mechanismBounds;
		
		
		this.startPosition = startMotionState.getPosition();
		this.startVelocity = startMotionState.getVelocity();
		this.endPosition = endMotionState.getPosition();
		this.endVelocity = endMotionState.getVelocity();
		this.maxAcceleration = velocityConstraints.getMaxAcceleration();
		this.maxDeceleration = velocityConstraints.getMaxDeceleration();
		this.maxVelocity = velocityConstraints.getMaxVelocity();
		this.minPosition = mechanismBounds.getMinPosition();
		this.maxPosition = mechanismBounds.getMaxPosition();
		
		this.reversed = endPosition < startPosition;
		double currentSetpoint;
		currentSetpoint = Math.abs(startPosition - endPosition);
		if (reversed) {
			this.endVelocity = -endVelocity;
			this.startVelocity = -startVelocity;
		}
		//Since we are dealing with non-zero start and end velocity values, we need to first figure out where the
		//motion profile gets to with the non-zero values and adjust the setpoint so it reaches the actual position
		//that it wants to get to.
		
		//Initial motion profile calculation with setpoint
		calculateMotionProfile(currentSetpoint);
		
		//Get the difference in setpoint between the input and the final one
		double finalPosition = getPositionAtTime(tTotal);
		
		double setpointDifference = endMotionState.getPosition() - finalPosition;
		
		//Recalculate the motion profile with the adjusted setpoint
		// calculateMotionProfile(endMotionState.getPosition() + setpointDifference);
	}
	
	private void calculateMotionProfile(double currentSetpoint) {
		//To calculate the motion profile, we first start by calculating the theoretical maximum velocity of the motion
		//profile if the velocity was allowed to reach infinity. This creates a triangular motion profile that only
		//contains an acceleration and deceleration period, no cruise since there is no maximum velocity.
		
		//The theoretical maximum velocity is then used to determine if the motion profile will ever reach the actual
		//maximum velocity, either becoming trapezoidal with a cruise period or triangular if it will never reach cruise
		//in time.
		
		//For simplicity we add on the extra distance and time it would take if the velocity started and ended at zero,
		//although the actual motion profile might have a nonzero start or end velocity
		
		//Find the time and distance it would take to get from zero velocity to the actual starting velocity of this motion profile.
		double zeroToStartVelocityTime = startVelocity / maxAcceleration;
		double zeroToStartVelocityDistance = zeroToStartVelocityTime * zeroToStartVelocityTime * maxAcceleration / 2;
		
		//Find the time and distance it would take to get from the actual ending velocity to zero velocity of this motion profile.
		double endVelocityToZeroTime = endVelocity / maxDeceleration;
		double endVelocityToZeroDistance = endVelocityToZeroTime * endVelocityToZeroTime * maxDeceleration / 2;
		
		//Find the theoretical total distance of the motion profile if the ends were added on (if the motion profile started and ended at zero velocity)
		double totalDistanceWithEnds = zeroToStartVelocityDistance + currentSetpoint + endVelocityToZeroDistance;
		
		double triangleDAccel;
		double triangleDDecel;
		
		//Find the acceleration and deceleration distance of the theoretical triangular motion profile if it were to start and end at zero velocity
		if (maxAcceleration >= maxDeceleration) {
			double accelDecelRatio = (maxAcceleration / (maxDeceleration + maxAcceleration));
			triangleDAccel = totalDistanceWithEnds * accelDecelRatio;
			triangleDDecel = totalDistanceWithEnds - triangleDAccel;
		} else {
			double accelDecelRatio = (maxDeceleration / (maxAcceleration + maxDeceleration));
			triangleDDecel = totalDistanceWithEnds * accelDecelRatio;
			triangleDAccel = totalDistanceWithEnds - triangleDDecel;
		}
		
		//Find the theoretical maximum velocity of the motion profile if it were allowed to reach infinite velocity
		double theoreticalMaxVelocity = Math.sqrt(2 * maxAcceleration * triangleDAccel);
		
		
		double tAccel, tDecel, dAccel, dDecel, dCruise, tCruise, tTotal;
		
		//If the starting velocity is greater than or equal to the theoretical max velocity
		if (startVelocity >= theoreticalMaxVelocity) {
			//Make the theoretical max velocity the starting velocity instead
			theoreticalMaxVelocity = startVelocity;
			
			//Make sure theoretical max velocity never goes above the actual max velocity
			maxVelocity = Math.min(theoreticalMaxVelocity, maxVelocity);
			
			//Since the start velocity is greater than or equal to the max velocity, it will never accelerate more than it is.
			tAccel = 0;
			dAccel = 0;
			
			//Calculate the deceleration and cruise as normal
			tDecel = maxVelocity / maxDeceleration;
			dDecel = (maxDeceleration * tDecel * tDecel) / 2;
			dCruise = totalDistanceWithEnds - triangleDAccel - dDecel;
			tCruise = dCruise / maxVelocity;
		}
		//If the ending velocity is greater than or equal to the theoretical max velocity
		else if (endVelocity >= theoreticalMaxVelocity) {
			//Make the theoretical max velocity the ending velocity velocity instead
			theoreticalMaxVelocity = endVelocity;
			
			//Make sure theoretical max velocity never goes above the actual max velocity
			maxVelocity = Math.min(theoreticalMaxVelocity, maxVelocity);
			
			//Since the ending velocity is greater than or equal to the max velocity, it will never decelerate.
			tDecel = 0;
			dDecel = 0;
			
			//Calculate the acceleration and cruise as normal
			tAccel = maxVelocity / maxAcceleration;
			dAccel = (maxAcceleration * tAccel * tAccel) / 2;
			dCruise = totalDistanceWithEnds - dAccel - triangleDDecel;
			tCruise = dCruise / maxVelocity;
			
		}
		//If the start and end velocity are below the theoretical max velocity (a regular trapezoidal profile)
		else {
			//Make sure theoretical max velocity never goes above the actual max velocity
			maxVelocity = Math.min(theoreticalMaxVelocity, maxVelocity);
			
			//Calculate the acceleration, deceleration, and cruise as normal
			tAccel = maxVelocity / maxAcceleration;
			tDecel = maxVelocity / maxDeceleration;
			dAccel = (maxAcceleration * tAccel * tAccel) / 2;
			dDecel = (maxDeceleration * tDecel * tDecel) / 2;
			dCruise = totalDistanceWithEnds - dAccel - dDecel;
			tCruise = dCruise / maxVelocity;
		}
		
		//Subtract the extra end times from the final acceleration and deceleration times
		tAccel = tAccel - zeroToStartVelocityTime;
		tDecel = tDecel - endVelocityToZeroTime;
		
		
		//Subtract the extra end distances from the final acceleration and deceleration distances
		dAccel = dAccel - zeroToStartVelocityDistance;
		dDecel = dDecel - endVelocityToZeroDistance;
		
		//If any of of the times or distances are less than 0, set them to 0.
		if (tCruise < 0 || dCruise < 0) {
			dCruise = 0;
			tCruise = 0;
		}
		if (tAccel < 0 || dAccel < 0) {
			tAccel = 0;
			dAccel = 0;
		}
		if (tDecel < 0 || dDecel < 0) {
			tDecel = 0;
			dDecel = 0;
		}
		
		
		//Find the total time of the motion profile
		tTotal = tAccel + tCruise + tDecel;
		this.tTotal = tTotal;
		
		double d = maxVelocity;
		double a = startVelocity;
		double c = tAccel;
		double h = endVelocity;
		double g = tTotal;
		double f = tCruise + tAccel;
		
		//Set the mathematical functions for the motion segments
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
		
		//Set the motion segments
		accelerationSegment = new MotionSegment(tAccel, dAccel, accelerationFunction);
		cruiseSegment = new MotionSegment(tCruise, dCruise, cruiseFunction);
		decelerationSegment = new MotionSegment(tDecel, dDecel, decelerationFunction);
	}
	
	/**
	 * Calculates the {@link MotionState} at a certain time.
	 *
	 * @param t time of the motion frame
	 * @return a new {@link MotionState} at time t
	 */
	public MotionState getFrameAtTime(double t) {
		//Get the velocity, position, and acceleration at the time
		double velocity = getVelocityAtTime(t);
		double position = getPositionAtTime(t);
		double acceleration = getAccelerationAtTime(t, velocity);
		
		//Make sure the position does not exceed the bounds
		if (!(minPosition == 0 && maxPosition == 0)) {
			position = Math.min(position, maxPosition);
			position = Math.max(position, minPosition);
		}
		
		//Check if it is finished
		isFinished = t <= tTotal;
		
		if (t < tTotal) {
			return new MotionState(position, velocity, acceleration, t);
		} else {
			return new MotionState(endPosition, endVelocity, acceleration, t);
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
		
		if (reversed) {
			return -output;
		}
		return output;
	}
	
	/**
	 * Returns the velocity of the motion profile at a certain position.
	 *
	 * @param position the position where the desired velocity value is.
	 * @return the velocity of the motion profile at a certain position.
	 */
	public double getVelocityAtPosition(double position) {
		if (position < accelerationSegment.getDistance()) {
			return Math.sqrt(startMotionState.getVelocity() * startMotionState.getVelocity() + 2 * maxAcceleration * position);
		} else if (position < accelerationSegment.getDistance() + cruiseSegment.getDistance()) {
			return maxVelocity;
		} else if (position < accelerationSegment.getDistance() + cruiseSegment.getDistance() + decelerationSegment.getDistance()) {
			position = position - accelerationSegment.getDistance() - cruiseSegment.getDistance();
			return Math.sqrt(maxVelocity * maxVelocity - 2 * maxDeceleration * position);
		} else {
			return 0;
		}
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
		
		if (reversed) {
			output = -output;
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
		if (position < accelerationSegment.getDistance()) {
			return Math.sqrt((2 * position) / maxAcceleration);
		} else if (position > accelerationSegment.getDistance() && position < cruiseSegment.getDistance()) {
			return accelerationSegment.getTime() +
					(position - accelerationSegment.getDistance()) / maxVelocity;
		} else {
			return accelerationSegment.getTime() +
					cruiseSegment.getTime() +
					Math.sqrt((2 * (position - accelerationSegment.getDistance() - cruiseSegment.getDistance())) / maxAcceleration);
		}
	}
	
	//Keep track of the previous velocity and time for getting acceleration
	double prevVelocity;
	double prevTime;
	
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
		
		if (reversed) {
			return -acceleration;
		}
		
		return acceleration;
	}
	
	public double getTotalTime() {
		return tTotal;
	}
	
	public MotionState getStartMotionState() {
		return startMotionState;
	}
	
	public MotionState getEndMotionState() {
		return endMotionState;
	}
	
	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}
	
	public MechanismBounds getMechanismBounds() {
		return mechanismBounds;
	}
	
	public boolean isFinished() {
		return isFinished;
	}
	
	public boolean isReversed() {
		return reversed;
	}
}