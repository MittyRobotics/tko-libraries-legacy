package com.amhsrobotics.libs.util.geometry;

import com.amhsrobotics.libs.datatypes.DrivetrainVelocities;

public class DifferentialDriveKinematics {

    private static DifferentialDriveKinematics instance = new DifferentialDriveKinematics();

    public static DifferentialDriveKinematics getInstance() {
        return instance;
    }

    private double TRACK_WIDTH;

    private DifferentialDriveKinematics(){

    }

    public DrivetrainVelocities getDrivetrainVelocitiesFromRadius(double robotVelocity, double radius){
        double angularVelocity = robotVelocity / robotVelocity;

        return new DrivetrainVelocities(angularVelocity * (radius - (TRACK_WIDTH / 2)),angularVelocity * (radius + (TRACK_WIDTH / 2)));

    }

    public double getTRACK_WIDTH() {
        return TRACK_WIDTH;
    }

    public void setTRACK_WIDTH(double TRACK_WIDTH) {
        this.TRACK_WIDTH = TRACK_WIDTH;
    }
}
