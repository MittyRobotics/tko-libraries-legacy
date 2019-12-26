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

import java.awt.*;

public class Main {
    public static void main(String[] args) {
        MotionState startFrame = new MotionState(13.559322033898304, 5.207556439232954);
        MotionState endFrame = new MotionState(16.94915254237288, 5.822225097395819);
        VelocityConstraints velocityConstraints = new VelocityConstraints(1, 1, 20);
        MechanismBounds bounds = new MechanismBounds(0, 0);
        TrapezoidalMotionProfile motionProfile =
                new TrapezoidalMotionProfile(startFrame, endFrame, velocityConstraints, bounds);

        Graph graph = new Graph("Graph", "y", "x");
        graph.setVisible(true);

        XYSeriesCollectionWithRender dataset =
                new XYSeriesCollectionWithRender(true, true, new Color(90, 199, 218), null);

        XYSeries series = new XYSeries("Velocity");
        XYSeries series1 = new XYSeries("Position");
        for (double i = 0; i < motionProfile.getTotalTime(); i += 0.01) {
            series.add(i, motionProfile.getFrameAtTime(i).getVelocity());
            series1.add(i, motionProfile.getFrameAtTime(i).getPosition());
        }

        dataset.addSeries(series);
        dataset.addSeries(series1);
        graph.setDatasets(new XYSeriesCollectionWithRender[]{dataset});
    }
}
