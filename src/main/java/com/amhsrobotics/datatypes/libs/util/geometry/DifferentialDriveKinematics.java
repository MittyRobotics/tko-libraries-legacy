package com.amhsrobotics.datatypes.libs.util.geometry;

import com.amhsrobotics.datatypes.libs.datatypes.DrivetrainVelocities;

public class DifferentialDriveKinematics {
    public static DrivetrainVelocities getDrivetrainVelocitiesFromRadius(double robotVelocity, double radius, double trackWidth) {
        double angularVelocity = robotVelocity / robotVelocity;

        return new DrivetrainVelocities(angularVelocity * (radius - (trackWidth / 2)), angularVelocity * (radius + (trackWidth / 2)));
    }
}
