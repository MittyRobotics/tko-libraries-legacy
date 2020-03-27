package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.MotionState;

public class SCurveProfileV2 {
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

    public SCurveProfileV2(MotionState startPoint, MotionState setpoint, double maxAcceleration, double maxDeceleration,
                           double maxJerk, double maxVelocity,
                           OverrideMethod overrideMethod) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxJerk = maxJerk;
        this.maxVelocity = maxVelocity;
        this.overrideMethod = overrideMethod;
        calculateMotionProfile(startPoint, setpoint);
    }

    public void calculateMotionProfile(MotionState startPoint, MotionState setpoint) {
        TrapezoidTimeSegment accelerationTrapezoid =
                calculateTrapezoid(maxVelocity - startPoint.getVelocity(), startPoint.getAcceleration(), 0,
                        maxAcceleration);
        TrapezoidTimeSegment decelerationTrapezoid =
                calculateTrapezoid(maxVelocity - setpoint.getVelocity(), 0, setpoint.getAcceleration(),
                        maxDeceleration);
        double tAAccel = accelerationTrapezoid.getTAccel();
        double tACruise = accelerationTrapezoid.getTCruise();
        double tADecel = accelerationTrapezoid.getTDecel();
        double tDAccel = decelerationTrapezoid.getTAccel();
        double tDCruise = decelerationTrapezoid.getTCruise();
        double tDDecel = decelerationTrapezoid.getTDecel();
        MotionSegment aaSegment = new MotionSegment(new Line(maxJerk, startPoint.getAcceleration()), 0, tAAccel,
                startPoint.getVelocity(), startPoint.getPosition());
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

        double tCruise = (setpoint.getPosition() - totalPos) / maxVelocity;
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

        if (totalPos > setpoint.getPosition()) {

        }


        System.out.println(
                "Times: " + tAAccel + " " + tACruise + " " + tADecel + " " + tCruise + " " + tDAccel + " " + tDCruise +
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

    public MotionState calculateState(double t) {
        MotionSegment[] segments = new MotionSegment[]{aaSegment, acSegment, adSegment, cruiseSegment, daSegment,
                dcSegment,
                ddSegment};
        return new MotionState(getPositionFromSegments(t, segments), getVelocityFromSegments(t, segments),
                getAccelerationFromSegments(t, segments));
    }

    private double getPositionFromSegments(double t, MotionSegment[] segments) {
        MotionSegment segment = identifySegment(t, segments);
        return segment.getPositionFromTime(t - segment.getStartTime());
    }

    private double getVelocityFromSegments(double t, MotionSegment[] segments) {
        MotionSegment segment = identifySegment(t, segments);
        return segment.getVelocityFromTime(t - segment.getStartTime());
    }

    private double getAccelerationFromSegments(double t, MotionSegment[] segments) {
        MotionSegment segment = identifySegment(t, segments);
        return segment.getAccelerationFromTime(t - segment.getStartTime());
    }

    private MotionSegment identifySegment(double t, MotionSegment[] segments) {
        MotionSegment segment = new MotionSegment(new Line(0, 0), 0, 0, 0,0);
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

    private TrapezoidTimeSegment calculateTrapezoid(double velocitySetpoint, double startAcceleration,
                                                    double endAcceleration, double localMaxAccel) {
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

        double theoreticalMaxAcceleration = Math.sqrt(2 * localMaxAccel * triangleDAccel);

        double tAccel, tDecel, dAccel, dDecel, dCruise, tCruise;

        if (startAcceleration >= theoreticalMaxAcceleration) {
            theoreticalMaxAcceleration = startAcceleration;

            theoreticalMaxAcceleration = Math.min(theoreticalMaxAcceleration, localMaxAccel);

            tAccel = 0;
            dAccel = 0;

            //Calculate the deceleration and cruise as normal
            tDecel = theoreticalMaxAcceleration / maxJerk;
            dDecel = (maxJerk * tDecel * tDecel) / 2;
            dCruise = totalDistanceWithEnds - triangleDAccel - dDecel;
            tCruise = dCruise / theoreticalMaxAcceleration;
        }
        //If the ending velocity is greater than or equal to the theoretical max velocity
        else if (endAcceleration >= theoreticalMaxAcceleration) {
            //Make the theoretical max velocity the ending velocity velocity instead
            theoreticalMaxAcceleration = endAcceleration;

            //Make sure theoretical max velocity never goes above the actual max velocity
            theoreticalMaxAcceleration = Math.min(theoreticalMaxAcceleration, localMaxAccel);

            //Since the ending velocity is greater than or equal to the max velocity, it will never decelerate.
            tDecel = 0;
            dDecel = 0;

            //Calculate the acceleration and cruise as normal
            tAccel = theoreticalMaxAcceleration / maxJerk;
            dAccel = (maxJerk * tAccel * tAccel) / 2;
            dCruise = totalDistanceWithEnds - dAccel - triangleDDecel;
            tCruise = dCruise / theoreticalMaxAcceleration;

        }
        //If the start and end velocity are below the theoretical max velocity (a regular trapezoidal profile)
        else {
            //Make sure theoretical max velocity never goes above the actual max velocity
            theoreticalMaxAcceleration = Math.min(theoreticalMaxAcceleration, localMaxAccel);

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
