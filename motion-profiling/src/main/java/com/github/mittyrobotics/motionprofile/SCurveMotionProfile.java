package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.MotionState;

public class SCurveMotionProfile {
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxJerk;
    private double maxVelocity;
    private OverrideMethod overrideMethod;
    private MotionSegment aaSegment;
    private MotionSegment acSegment;
    private MotionSegment adSegment;
    private MotionSegment cruiseSegment;
    private MotionSegment daSegment;
    private MotionSegment dcSegment;
    private MotionSegment ddSegment;
    private boolean reversed = false;
    private MotionState startState;
    private MotionState endState;

    /**
     * Creates a new {@link SCurveMotionProfile}. The s-curve motion profile is a more complex form of the
     * trapezoidal motion profile, resulting in a trapezoidal acceleration over time shape versus a trapezoidal
     * velocity over time shape, and overall resulting in smoother motion with the tradeoff of a slightly greater
     * travel time.
     * <p>
     * In s-curve motion profiles, because the acceleration over time graph is trapezoidal, there is an overall reduced
     * amount of jerk in the motion versus trapezoidal motion profiles.
     * <p>
     * The motion profile also supports handling cases where the end state is impossible to reach from the system's
     * start state. This is the override method, and the types of override include: overshooting the end state position
     * and coming back within the bounds of the system's velocity and acceleration constraints, violating the system's
     * velocity and acceleration constraints to reach the end state position on time, or stopping as close to the
     * end state as possible within the velocity and acceleration constraints without coming back.
     *
     * @param startState      the starting {@link MotionState} of the motion profile.
     * @param endState        the ending {@link MotionState} of the motion profile.
     * @param maxAcceleration the maximum acceleration of the motion profile. (units/s^2)
     * @param maxDeceleration the maximum deceleration of the motion profile. (units/s^2)
     * @param maxJerk         the maximum jerk of the motion profile. (units/s^3)
     * @param maxVelocity     the maximum velocity of the motion profile. (units/s)
     * @param overrideMethod  the method to override the motion profile if the endState is impossible to reach from the
     *                        system's start state.
     */
    public SCurveMotionProfile(MotionState startState, MotionState endState, double maxAcceleration,
                               double maxDeceleration,
                               double maxJerk, double maxVelocity, OverrideMethod overrideMethod) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxJerk = maxJerk;
        this.maxVelocity = maxVelocity;
        this.overrideMethod = overrideMethod;

        generateMotionProfile(startState, endState, maxVelocity);
    }

    /**
     * Generates the motion profile.
     * <p>
     * Breaks up the motion profile in it's seven acceleration segments: acceleration acceleration, acceleration
     * cruise, acceleration deceleration, cruise, deceleration acceleration, deceleration cruise, and deceleration
     * deceleration. Each acceleration segment is treated as a line in the form of y=mx+b, and the first and second
     * integral equations for y=mx+b are used to determine the velocity and position given an input of time in the
     * <code>calculateState(double t)</code> function.
     *
     * @param startState         the starting {@link MotionState} of the motion profile
     * @param endState           the ending {@link MotionState} of the motion profile
     * @param currentMaxVelocity the current max velocity value, used for recursively lowering max velocity in the
     *                           case of the total distance taking less time than it takes to fully accelerate and
     *                           decelerate.
     */
    public void generateMotionProfile(MotionState startState, MotionState endState, double currentMaxVelocity) {
        this.startState =
                new MotionState(startState.getPosition(), startState.getVelocity(), startState.getAcceleration());
        this.endState = new MotionState(endState.getPosition(), endState.getVelocity(), endState.getAcceleration());

        reversed = startState.getPosition() > endState.getPosition();

        if (reversed) {
            startState.setPosition(this.endState.getPosition());
            endState.setPosition(this.startState.getPosition());
            startState.setVelocity(-startState.getVelocity());
            endState.setVelocity(-endState.getVelocity());
            startState.setAcceleration(-startState.getAcceleration());
        }
        else{
            endState.setAcceleration(-endState.getAcceleration());
        }

        TrapezoidTimeSegment accelerationTrapezoid =
                calculateTrapezoid(currentMaxVelocity - startState.getVelocity(), startState.getAcceleration(), 0,
                        maxAcceleration);
        TrapezoidTimeSegment decelerationTrapezoid =
                calculateTrapezoid(currentMaxVelocity - endState.getVelocity(), 0, endState.getAcceleration(),
                        maxDeceleration);
        double tAAccel = accelerationTrapezoid.getTAccel();
        double tACruise = accelerationTrapezoid.getTCruise();
        double tADecel = accelerationTrapezoid.getTDecel();

        if (startState.getVelocity() >= maxVelocity) {
            tAAccel = 0;
            tACruise = 0;
            tADecel = 0;
        }

        double tDAccel = decelerationTrapezoid.getTAccel();
        double tDCruise = decelerationTrapezoid.getTCruise();
        double tDDecel = decelerationTrapezoid.getTDecel();

        if (endState.getVelocity() >= maxVelocity) {
            tDAccel = 0;
            tDCruise = 0;
            tDDecel = 0;
        }

        MotionSegment aaSegment = new MotionSegment(new Line(maxJerk, startState.getAcceleration()), 0, tAAccel,
                startState.getVelocity(), startState.getPosition());
        MotionSegment acSegment =
                new MotionSegment(new Line(0, accelerationTrapezoid.getMaxAcceleration()), aaSegment.getEndTime(),
                        tACruise,
                        aaSegment.getVelocity(), aaSegment.getPosition());
        MotionSegment adSegment = new MotionSegment(new Line(-maxJerk, accelerationTrapezoid.getMaxAcceleration()),
                acSegment.getEndTime(),
                tADecel,
                acSegment.getVelocity(), acSegment.getPosition());
        MotionSegment daSegment = new MotionSegment(new Line(-maxJerk, 0), adSegment.getEndTime(), tDAccel,
                adSegment.getVelocity(), adSegment.getPosition());
        MotionSegment dcSegment =
                new MotionSegment(new Line(0, -decelerationTrapezoid.getMaxAcceleration()), daSegment.getEndTime(),
                        tDCruise,
                        daSegment.getVelocity(), daSegment.getPosition());
        MotionSegment ddSegment =
                new MotionSegment(new Line(maxJerk, -decelerationTrapezoid.getMaxAcceleration()),
                        dcSegment.getEndTime(), tDDecel,
                        dcSegment.getVelocity(), dcSegment.getPosition());
        double totalPos = ddSegment.getPosition();

        double tCruise = (endState.getPosition() - totalPos) / currentMaxVelocity;
        MotionSegment cruiseSegment;
        if (tCruise > 0) {
            cruiseSegment = new MotionSegment(new Line(0, 0), adSegment.getEndTime(), tCruise,
                    adSegment.getVelocity(), adSegment.getPosition());

            daSegment.setStartTime(cruiseSegment.getEndTime());
            dcSegment.setStartTime(daSegment.getEndTime());
            ddSegment.setStartTime(dcSegment.getEndTime());
            daSegment.setV0(cruiseSegment.getVelocity());
            dcSegment.setV0(daSegment.getVelocity());
            ddSegment.setV0(dcSegment.getVelocity());
            daSegment.setX0(cruiseSegment.getPosition());
            dcSegment.setX0(daSegment.getPosition());
            ddSegment.setX0(dcSegment.getPosition());
        } else {
            cruiseSegment = new MotionSegment(new Line(0, 0), 0, 0, 0, 0);
        }

        if (totalPos > endState.getPosition()) {
            generateMotionProfile(this.startState, this.endState, currentMaxVelocity - 1);
        } else {
            System.out.println(
                    "Times: " + tAAccel + " " + tACruise + " " + tADecel + " " + tCruise + " " + tDAccel + " " +
                            tDCruise +
                            " " + tDDecel);
            System.out.println("Positions: " + aaSegment.getPosition() + " " + acSegment.getPosition() + " " +
                    adSegment.getPosition() + " " + cruiseSegment.getPosition() + " " + daSegment.getPosition() + " " +
                    dcSegment.getPosition() + " " + ddSegment.getPosition());
            System.out.println("Velocities: " + aaSegment.getVelocity() + " " + acSegment.getVelocity() + " " +
                    adSegment.getVelocity() + " " + cruiseSegment.getVelocity() + " " + daSegment.getVelocity() + " " +
                    dcSegment.getVelocity() + " " + ddSegment.getVelocity());

            this.aaSegment = aaSegment;
            this.acSegment = acSegment;
            this.adSegment = adSegment;
            this.cruiseSegment = cruiseSegment;
            this.daSegment = daSegment;
            this.dcSegment = dcSegment;
            this.ddSegment = ddSegment;
        }
    }

    /**
     * Calculates the {@link MotionState} at time <code>t</code>.
     * <p>
     * This starts by finding which {@link MotionSegment} of the motion profile the time is in. It then gets the
     * the acceleration from the acceleration line of that segment, the velocity from the first integral of the
     * acceleration line of that segment, and the position from the second integral of the acceleration line of that
     * segment.
     *
     * @param t the time to find the {@link MotionState} at.
     * @return the {@link MotionState} at time <code>t</code>.
     */
    public MotionState calculateState(double t) {
        MotionSegment[] segments = new MotionSegment[]{aaSegment, acSegment, adSegment, cruiseSegment, daSegment,
                dcSegment,
                ddSegment};
        return new MotionState(getPositionFromSegments(t, segments), getVelocityFromSegments(t, segments),
                getAccelerationFromSegments(t, segments));
    }

    /**
     * Returns the position at time <code>t</code> from a time-ordered array of {@link MotionSegment}s.
     *
     * @param t        time
     * @param segments array of {@link MotionSegment}s. Must be in sequential order based on time.
     * @return the position at time <code>t</code> from the segments
     */
    private double getPositionFromSegments(double t, MotionSegment[] segments) {
        MotionSegment segment = identifySegment(t, segments);
        if (t >= ddSegment.getEndTime()) {
            return endState.getPosition();
        }
        if (reversed) {
            System.out.println(startState.getPosition());
            return startState.getPosition() - segment.getPositionFromTime(t - segment.getStartTime()) +
                    endState.getPosition();
        }
        return segment.getPositionFromTime(t - segment.getStartTime());
    }

    /**
     * Returns the velocity at time <code>t</code> from a time-ordered array of {@link MotionSegment}s.
     *
     * @param t        time
     * @param segments array of {@link MotionSegment}s. Must be in sequential order based on time.
     * @return the velocity at time <code>t</code> from the segments
     */
    private double getVelocityFromSegments(double t, MotionSegment[] segments) {
        MotionSegment segment = identifySegment(t, segments);
        if (reversed) {
            return -segment.getVelocityFromTime(t - segment.getStartTime());
        }
        return segment.getVelocityFromTime(t - segment.getStartTime());
    }

    /**
     * Returns the acceleration at time <code>t</code> from a time-ordered array of {@link MotionSegment}s.
     *
     * @param t        time
     * @param segments array of {@link MotionSegment}s. Must be in sequential order based on time.
     * @return the acceleration at time <code>t</code> from the segments
     */
    private double getAccelerationFromSegments(double t, MotionSegment[] segments) {
        MotionSegment segment = identifySegment(t, segments);
        if (reversed) {
            return -segment.getAccelerationFromTime(t - segment.getStartTime());
        }
        return segment.getAccelerationFromTime(t - segment.getStartTime());
    }

    /**
     * Identifies which segment of the motion profile time <code>t</code> is in given a time-ordered input array of
     * {@link MotionSegment}s.
     *
     * @param t        time
     * @param segments array of {@link MotionSegment}s. Must be in sequential order based on time.
     * @return the {@link MotionSegment} that time <code>t</code> is in.
     */
    private MotionSegment identifySegment(double t, MotionSegment[] segments) {
        MotionSegment segment = new MotionSegment(new Line(0, 0), 0, 0, 0, 0);
        for (int i = 0; i < segments.length; i++) {
            MotionSegment s = segments[i];
            double minTime = s.getStartTime();
            double maxTime = s.getEndTime();
            if (t >= minTime && t <= maxTime) {
                segment = s;
            }
        }
        return segment;
    }

    /**
     * Calculates the {@link TrapezoidTimeSegment} given the velocity setpoint, start acceleration, end acceleration,
     * and max velocity. The {@link TrapezoidTimeSegment} contains the acceleration, cruise, and deceleration times
     * in a trapezoidal acceleration graph to reach the velocity setpoint from a start and end acceleration.
     *
     * @param velocitySetpoint  the total change in velocity that must be covered by the acceleration trapezoid.
     * @param startAcceleration the starting acceleration of the acceleration trapezoid.
     * @param endAcceleration   the ending acceleration of the acceleration trapezoid.
     * @param macAccel          the max acceleration for the acceleration trapezoid to reach.
     * @return the {@link TrapezoidTimeSegment} from the input parameters.
     */
    private TrapezoidTimeSegment calculateTrapezoid(double velocitySetpoint, double startAcceleration,
                                                    double endAcceleration, double macAccel) {
        double zeroToStartAccelerationTime = startAcceleration / maxJerk;
        double zeroToStartAccelerationDistance =
                zeroToStartAccelerationTime * zeroToStartAccelerationTime * maxJerk / 2;

        double endAccelerationToZeroTime = endAcceleration / maxJerk;
        double endAccelerationToZeroDistance = endAccelerationToZeroTime * endAccelerationToZeroTime * maxJerk / 2;

        double totalDistanceWithEnds =
                zeroToStartAccelerationDistance + velocitySetpoint + endAccelerationToZeroDistance;

        double triangleDAccel;
        double triangleDDecel;

        double accelDecelRatio = .5;
        triangleDDecel = totalDistanceWithEnds * accelDecelRatio;
        triangleDAccel = totalDistanceWithEnds - triangleDDecel;

        double theoreticalMaxAcceleration = Math.sqrt(2 * macAccel * triangleDAccel);

        double tAccel, tDecel, dAccel, dDecel, dCruise, tCruise;

        if (startAcceleration >= theoreticalMaxAcceleration) {
            theoreticalMaxAcceleration = startAcceleration;

            theoreticalMaxAcceleration = Math.min(theoreticalMaxAcceleration, macAccel);

            tAccel = 0;
            dAccel = 0;

            //Calculate the deceleration and cruise as normal
            tDecel = theoreticalMaxAcceleration / maxJerk;
            dDecel = (maxJerk * tDecel * tDecel) / 2;
            dCruise = totalDistanceWithEnds - triangleDAccel - dDecel;
            tCruise = dCruise / theoreticalMaxAcceleration;
        }
        //If the ending acceleration is greater than or equal to the theoretical max acceleration
        else if (endAcceleration >= theoreticalMaxAcceleration) {
            //Make the theoretical max acceleration the ending acceleration instead
            theoreticalMaxAcceleration = endAcceleration;

            //Make sure theoretical max acceleration never goes above the actual max acceleration
            theoreticalMaxAcceleration = Math.min(theoreticalMaxAcceleration, macAccel);

            //Since the ending acceleration is greater than or equal to the max acceleration, it will never decelerate.
            tDecel = 0;
            dDecel = 0;

            //Calculate the acceleration and cruise as normal
            tAccel = theoreticalMaxAcceleration / maxJerk;
            dAccel = (maxJerk * tAccel * tAccel) / 2;
            dCruise = totalDistanceWithEnds - dAccel - triangleDDecel;
            tCruise = dCruise / theoreticalMaxAcceleration;

        }
        //If the start and end acceleration are below the theoretical max acceleration (a regular trapezoidal profile)
        else {
            //Make sure theoretical max acceleration never goes above the actual max acceleration
            theoreticalMaxAcceleration = Math.min(theoreticalMaxAcceleration, macAccel);

            //Calculate the acceleration, deceleration, and cruise as normal
            tAccel = theoreticalMaxAcceleration / maxJerk;
            tDecel = theoreticalMaxAcceleration / maxJerk;
            dAccel = (maxJerk * tAccel * tAccel) / 2;
            dDecel = (maxJerk * tDecel * tDecel) / 2;
            dCruise = totalDistanceWithEnds - dAccel - dDecel;
            tCruise = dCruise / theoreticalMaxAcceleration;
        }

        //Subtract the extra end times from the final acceleration and deceleration times
        tAccel = tAccel - zeroToStartAccelerationTime;
        tDecel = tDecel - endAccelerationToZeroTime;


        //Subtract the extra end distances from the final acceleration and deceleration distances
        dAccel = dAccel - zeroToStartAccelerationDistance;
        dDecel = dDecel - endAccelerationToZeroDistance;

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

        return new TrapezoidTimeSegment(tAccel, tCruise, tDecel, theoreticalMaxAcceleration);
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxJerk() {
        return maxJerk;
    }

    public OverrideMethod getOverrideMethod() {
        return overrideMethod;
    }
}
