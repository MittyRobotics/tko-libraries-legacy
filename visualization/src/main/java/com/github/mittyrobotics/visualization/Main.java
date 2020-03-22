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

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;

import java.awt.*;

public class Main {
    public static void main(String[] args) {
        Graph graph = new Graph();
        graph.scaleGraphToScale(.1, 0, 0);
        XYSeriesWithRenderer series = new XYSeriesWithRenderer("rectangle", Color.red, true, false, null);
        series = GraphUtil.populateSeries(series, GraphUtil.rectangle(new Transform(0, 0, 0), 5, 5));
        XYSeriesWithRenderer series1 = new XYSeriesWithRenderer("circle", Color.cyan, true, false, null);
        series1 = GraphUtil.populateSeries(series1, GraphUtil.circle(new Circle(new Position(10, 0), 5)));
        XYSeriesWithRenderer series2 = new XYSeriesWithRenderer("arc", Color.green, true, false, null);
        series2 = GraphUtil.populateSeries(series2, GraphUtil.arc(new ArcSegment(new Position(-10, 0),
                new Position(-14, 4), new Position(-15, 0))));
        graph.addSeries(series);
        graph.addSeries(series1);
        graph.addSeries(series2);
    }
}
