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

package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithParameter;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphUtil;
import com.github.mittyrobotics.visualization.XYSeriesWithRenderer;

import java.awt.*;

public class Main {
    public static void main(String[] args) {
        Path path = new Path(PathGenerator.generateQuinticHermiteSplinePath(new Transform[]{new Transform(0, 0, 0), new Transform(4, 4, 0), new Transform(0, 0, Math.PI)}));
        Graph graph = new Graph();
        double dt = 0.02;
        graph.addSeries(GraphUtil.populateSeries(new XYSeriesWithRenderer("Path", Color.gray, true, false, null), GraphUtil.parametric(path, .01, .1)));
        TransformWithParameter expectedPathTransform = new TransformWithParameter(path.getStartWaypoint(), 0);
        double t = 0;
        while(true){
            t = path.getParameterFromLength(path.getGaussianQuadratureLength(t) + 1*dt);
            Transform oldTransform = expectedPathTransform;
            expectedPathTransform = new TransformWithParameter(new Transform(path.getTransform(t)), t);
            graph.changeSeries("Point", GraphUtil.populateSeries(new XYSeriesWithRenderer("Point"), GraphUtil.arrow(expectedPathTransform, .1, .1)));
            System.out.println(oldTransform.getPosition().distance(expectedPathTransform.getPosition()) / dt);
//            System.out.println(path.getParameterFromLength(path.getGaussianQuadratureLength(t)) + " " + t + " " + (t-path.getParameterFromLength(path.getGaussianQuadratureLength(t))));
            try {
                Thread.sleep((long) (1000.0*dt));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
