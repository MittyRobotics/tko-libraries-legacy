/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.motionprofile.util.datatypes.MechanismBounds;
import com.github.mittyrobotics.visualization.graphs.Graph;
import com.github.mittyrobotics.visualization.util.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

import javax.swing.*;

public class Main {
    public static void main(String[] args) {
        MotionState startFrame = new MotionState(200, 0);
        MotionState endFrame = new MotionState(0, 0);

        VelocityConstraints velocityConstraints = new VelocityConstraints(5, 5, 20);

        MechanismBounds bounds = new MechanismBounds(0, 0);

        TrapezoidalMotionProfile motionProfile =
                new TrapezoidalMotionProfile(startFrame, endFrame, velocityConstraints, bounds);

        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                Graph graph = new Graph("Graph", "y", "x");
                graph.setVisible(true);

                XYSeriesCollectionWithRender dataset =
                        new XYSeriesCollectionWithRender(false, true, null, null);

                XYSeries series = new XYSeries("Velocity");
                XYSeries series1 = new XYSeries("Position");
                XYSeries series2 = new XYSeries("Acceleration");
                for (double i = 0; i < motionProfile.getTotalTime(); i += 0.01) {
                    series.add(i, motionProfile.getVelocityAtTime(i));
                    series1.add(i, motionProfile.getPositionAtTime(i));
                    series2.add(i, motionProfile.getAccelerationAtTime(i));
                }

                dataset.addSeries(series);
                dataset.addSeries(series1);
                dataset.addSeries(series2);
                graph.setDatasets(new XYSeriesCollectionWithRender[]{dataset});
            }
        });
    }
}
