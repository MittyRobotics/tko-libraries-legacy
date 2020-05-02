/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.visualization;

import com.github.mittyrobotics.datatypes.motion.SwerveDriveState;
import com.github.mittyrobotics.datatypes.motion.SwerveModuleState;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import org.jfree.data.xy.XYSeries;

import javax.swing.*;

public class RobotGraph extends Graph {
    private String robotKey = "Robot";
    private String module1Key = "module1";
    private String module2Key = "module2";
    private String module3Key = "module3";
    private String module4Key = "module4";
    private String pathKey = "Path";

    public RobotGraph() {
        super();
        XYSeries robotSeries = new XYSeries(robotKey, false);
        XYSeries module1Series = new XYSeries(module1Key, false);
        XYSeries module2Series = new XYSeries(module2Key, false);
        XYSeries module3Series = new XYSeries(module3Key, false);
        XYSeries module4Series = new XYSeries(module4Key, false);
        XYSeries pathSeries = new XYSeries(pathKey, false);

        scaleGraphToScale(1, 0, 0);

        addSeries(robotSeries);
        addSeries(module1Series);
        addSeries(module2Series);
        addSeries(module3Series);
        addSeries(module4Series);
        addSeries(pathSeries);
    }

    public void graphDifferentialDrive(Transform robotTransform, double robotWidth, double robotHeight) {
        //convert to inches
        changeSeries(robotKey, GraphUtil.populateSeries(new XYSeriesWithRenderer("robotKey"),
                GraphUtil.rectangle(robotTransform, robotWidth, robotHeight)));
    }

    public void graphSwerveDrive(Transform robotTransform, SwerveDriveState swerveModulePositions, double robotWidth,
                                 double robotHeight, double moduleWidth, double moduleHeight) {
        SwingUtilities.invokeLater(() -> {
            changeSeries(robotKey, GraphUtil.populateSeries(new XYSeriesWithRenderer("robotKey"),
                    GraphUtil.rectangle(robotTransform, robotWidth, robotHeight)));
            changeSeries(module1Key, GraphUtil.populateSeries(new XYSeriesWithRenderer("robotKey"),
                    GraphUtil.rectangle(robotTransform.add(new Transform(robotWidth / 2, -robotHeight / 2,
                            swerveModulePositions.getFrState().getSteerState())), moduleWidth, moduleHeight)));
            changeSeries(module2Key, GraphUtil.populateSeries(new XYSeriesWithRenderer("robotKey"),
                    GraphUtil.rectangle(robotTransform.add(new Transform(robotWidth / 2, robotHeight / 2,
                            swerveModulePositions.getFlState().getSteerState())), moduleWidth, moduleHeight)));
            changeSeries(module3Key, GraphUtil.populateSeries(new XYSeriesWithRenderer("robotKey"),
                    GraphUtil.rectangle(robotTransform.add(new Transform(-robotWidth / 2, robotHeight / 2,
                            swerveModulePositions.getBlState().getSteerState())), moduleWidth, moduleHeight)));
            changeSeries(module4Key, GraphUtil.populateSeries(new XYSeriesWithRenderer("robotKey"),
                    GraphUtil.rectangle(robotTransform.add(new Transform(-robotWidth / 2, -robotHeight / 2,
                            swerveModulePositions.getBrState().getSteerState())), moduleWidth, moduleHeight)));
        });

    }

    public void graphPath() {
    }
}
