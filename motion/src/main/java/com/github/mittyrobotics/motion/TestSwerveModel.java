/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.motion.SwerveModuleState;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.modeling.models.SwerveDriveModel;
import com.github.mittyrobotics.motion.modeling.motors.Falcon500Motor;
import com.github.mittyrobotics.motion.modeling.motors.NEOMotor;
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
