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

package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithVelocityAndCurvature;
import com.github.mittyrobotics.path.generation.splines.QuinticHermiteSpline;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphUtil;
import com.github.mittyrobotics.visualization.XYSeriesWithRenderer;

public class Main {
    public static void main(String[] args) {
        Graph graph = new Graph();

        graph.scaleGraphToScale(.15, 50, 25);

        graph.getChart().removeLegend();

        TransformWithVelocityAndCurvature p1 = new TransformWithVelocityAndCurvature(new Transform(0, 0, 0), 1, 0);
        TransformWithVelocityAndCurvature p2 = new TransformWithVelocityAndCurvature(new Transform(100, 50, 0), 0, 0);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(p1, p2);
        graph.addSeries(GraphUtil.populateSeries(XYSeriesWithRenderer.withLines("Series"), GraphUtil.parametric(spline,
                0.01, 2)));

        System.out.println(spline.getGaussianQuadratureLength());
        System.out.println(spline.getRawLength(1000, 0, 1));
        System.out.println(spline.getParameterFromLength(100, spline.getGaussianQuadratureLength()));
        System.out.println(spline.getRawLength(1000, 0, spline.getParameterFromLength(50, spline.getGaussianQuadratureLength())));
    }
}