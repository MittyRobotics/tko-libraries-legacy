package com.github.mittyrobotics.datatypes.motion;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import org.ejml.simple.SimpleMatrix;

public class SwerveDriveKinematics {
    /**
     * Solves the forward kinematics for a swerve drivetrain.
     * From equation 12.17 in https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
     *
     * @param l
     * @param w
     * @param frState
     * @param flState
     * @param blState
     * @param brState
     * @return
     */
    public static SwerveDriveState solveForwardKinematics(double l, double w, SwerveModuleState frState,
                                                          SwerveModuleState flState, SwerveModuleState blState,
                                                          SwerveModuleState brState) {
        //Create kinematics matrix and pseudoinverse it to solve for forward kinematics instead of inverse kinematics
        SimpleMatrix forwardKinematicsMatrix = createSwerveKinematicsMatrix(l, w).pseudoInverse();
        double[] inputMatrixVals = new double[8];
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{frState, flState, blState, brState};
        //Map swerve module states into the 8x1 module velocity vector matrix
        for (int i = 0; i < inputMatrixVals.length; i += 2) {
            double vx = Math.cos(swerveModuleStates[i / 2].getSteerState()) * swerveModuleStates[i / 2].getWheelState();
            double vy = Math.sin(swerveModuleStates[i / 2].getSteerState()) * swerveModuleStates[i / 2].getWheelState();
            inputMatrixVals[i] = vx;
            inputMatrixVals[i + 1] = vy;
        }
        SimpleMatrix inputMatrix = new SimpleMatrix(8, 1, true, inputMatrixVals);
        //Solve the forward kinematics
        SimpleMatrix solvedMatrix = forwardKinematicsMatrix.mult(inputMatrix);

        return new SwerveDriveState(solvedMatrix.get(0), solvedMatrix.get(1), solvedMatrix.get(2), frState, flState,
                blState, brState);
    }

    /**
     * Solves the forward kinematics for a swerve drivetrain.
     * From equation 12.17 in https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
     *
     * @param l
     * @param w
     * @param frVelocity
     * @param flVelocity
     * @param blVelocity
     * @param brVelocity
     * @param frAngle
     * @param flAngle
     * @param blAngle
     * @param brAngle
     * @return
     */
    public static SwerveDriveState solveForwardKinematics(double l, double w, double frVelocity, double flVelocity,
                                                          double blVelocity, double brVelocity, double frAngle,
                                                          double flAngle, double blAngle, double brAngle) {
        return SwerveDriveKinematics.solveForwardKinematics(l, w, new SwerveModuleState(frVelocity, frAngle),
                new SwerveModuleState(flVelocity, flAngle), new SwerveModuleState(blVelocity, blAngle),
                new SwerveModuleState(brVelocity, brAngle));
    }

    /**
     * Solves the inverse kinematics for a swerve drivetrain.
     * From equation 12.14 in https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
     *
     * @param l
     * @param w
     * @param velX
     * @param velY
     * @param velRad
     * @return
     */
    public static SwerveDriveState solveInverseKinematics(double l, double w, double velX, double velY, double velRad) {
        //Create swerve drive kinematics matrix
        SimpleMatrix inverseKinematicsMatrix = createSwerveKinematicsMatrix(l, w);
        //Create input matrix, a 3x1 matrix that contains the x, y, and angualr velocity
        SimpleMatrix inputMatrix = new SimpleMatrix(new double[][]{
                {velX},
                {velY},
                {velRad}
        });
        //Solve the inverse kinematics
        SimpleMatrix solvedMatrix = inverseKinematicsMatrix.mult(inputMatrix);

        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        //Map x and y velocity vectors to respective swerve module states
        for (int i = 0; i < solvedMatrix.getNumElements(); i += 2) {
            double vx = solvedMatrix.get(i);
            double vy = solvedMatrix.get(i + 1);
            double v = Math.sqrt(vx * vx + vy * vy);
            double theta = Math.atan2(vy, vx);
            swerveModuleStates[i / 2] = new SwerveModuleState(v, theta);
        }

        return new SwerveDriveState(velX, velY, velRad, swerveModuleStates[0], swerveModuleStates[1],
                swerveModuleStates[2], swerveModuleStates[3]);
    }

    /**
     * Creates the swerve drive 8x3 inverse kinematics matrix. Maps x, y, and angular velocity to individual swerve
     * module x and y velocity vectors.
     * From equation 12.14 in https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
     *
     * @param l
     * @param w
     * @return
     */
    public static SimpleMatrix createSwerveKinematicsMatrix(double l, double w) {
        double l2 = l / 2;
        double w2 = w / 2;
        double frX = l2;
        double frY = -w2;
        double flX = l2;
        double flY = w2;
        double blX = -l2;
        double blY = w2;
        double brX = -l2;
        double brY = -w2;
        return new SimpleMatrix(new double[][]{
                {1, 0, frY},
                {0, 1, frX},
                {1, 0, flY},
                {0, 1, flX},
                {1, 0, blY},
                {0, 1, blX},
                {1, 0, brY},
                {0, 1, brX}
        });
    }
}
