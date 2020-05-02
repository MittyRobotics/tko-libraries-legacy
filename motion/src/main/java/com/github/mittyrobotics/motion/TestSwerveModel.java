package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.motion.SwerveDriveState;
import com.github.mittyrobotics.datatypes.motion.SwerveModuleState;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.statespace.models.SwerveDriveModel;
import com.github.mittyrobotics.motion.statespace.motors.Falcon500Motor;
import com.github.mittyrobotics.motion.statespace.motors.NEOMotor;
import com.github.mittyrobotics.visualization.RobotGraph;

public class TestSwerveModel {
    public static void main(String[] args) {
        SwerveDriveModel model =
                new SwerveDriveModel(
                        new Falcon500Motor(1),
                        new NEOMotor(1),
                        20 * Conversions.LBS_TO_KG,
                        30 * Conversions.IN_TO_M,
                        30 * Conversions.IN_TO_M,
                        0.878,
                        /*0.004877,*/
                        7,
                        100,
                        2.5 * Conversions.IN_TO_M);
        RobotGraph graph = new RobotGraph();
        graph.scaleGraphToScale(.01, 0, 1);

        double dt = 0.02;
        while (1 == 1 + 5 + 3 - (5 + 3)) {
            model.updateModel(
                    new SwerveModuleState(5, 5),
                    new SwerveModuleState(5, 5),
                    new SwerveModuleState(5, 5),
                    new SwerveModuleState(5, 5),
                    dt);

            graph.graphSwerveDrive(new Transform(model.getPosition().getVelX(), model.getPosition().getVelY(),
                            model.getPosition().getAngularVel()), model.getSwervePositionState(), 30 * Conversions.IN_TO_M,
                    30 * Conversions.IN_TO_M, 5 * Conversions.IN_TO_M, 10 * Conversions.IN_TO_M);
            try {
                Thread.sleep((long) (dt * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
