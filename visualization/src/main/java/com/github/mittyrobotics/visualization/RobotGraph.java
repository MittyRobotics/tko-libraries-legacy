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

import com.github.mittyrobotics.datatypes.positioning.Transform;

public class RobotGraph extends Graph {

    private int lastIndex = 2;

    public RobotGraph() {
        super("Robot Graph", "y", "x");
        scaleGraphToScale(1,0,0);
        setSize(800, 800);
    }

    public void graphRobot(Transform robotTransform, double width, double length) {
        XYSeriesCollectionWithRenderer dataset = new XYSeriesCollectionWithRenderer();
        dataset.addSeries(GraphManager.getInstance().rectangleSeries(robotTransform, width, length, "robot"));
        dataset.addSeries(GraphManager.getInstance().arrowSeries(robotTransform, length / 2, 1, "robot Transform"));
        getPlot().setDataset(0, dataset);
    }

    @Override
    public void addDataset(XYSeriesCollectionWithRenderer dataset) {
        getPlot().setDataset(lastIndex, dataset);
        getPlot().setRenderer(lastIndex,
                new XYLineShapeColorRenderer(dataset.isShowPoints(), dataset.isShowLines(), dataset.getColor()));
        lastIndex++;
    }

    public void addPath(XYSeriesCollectionWithRenderer dataset) {
        getPlot().setDataset(1, dataset);
        getPlot().setRenderer(1,
                new XYLineShapeColorRenderer(dataset.isShowPoints(), dataset.isShowLines(), dataset.getColor()));
    }

    @Override
    public void clearGraph() {
        for (int i = 2; i < getPlot().getDatasetCount(); i++) {
            getPlot().setDataset(i, null);
        }
        lastIndex = 2;
    }
}