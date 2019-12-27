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

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.datatypes.positioning.TransformWithT;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.visualization.graphs.Graph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;

public class Main {
    public static void main(String[] args) {
        Graph graph = new Graph();

        graph.resizeGraph(-20, 120, -20, 120);

        graph.getChart().removeLegend();
        CubicHermitePath path = new CubicHermitePath(new Transform[]{
                new Transform(100, 100, 180),
                new Transform(0, 0, 180)

        });
        Transform transform = new Transform(0, 0);
        while (true) {

            TransformWithT closestPos = path.getClosestTransform(transform.getPosition(), 10, 3);
            Position targetPos = path.getClosestTransform(closestPos.getPosition(), 20, false, 10, 3).getPosition();

            Transform finalTransform = transform;
            SwingUtilities.invokeLater(new Runnable() {
                @Override
                public void run() {
                    graph.clearGraph();
                    graph.addDataset(
                            GraphManager.getInstance().graphParametric(path, 0.01, 2, 1, "spline", Color.cyan));
                    graph.addDataset(GraphManager.getInstance()
                            .graphArrow(new Transform(closestPos.getPosition(), 90), 5, 2, "asdf", Color.green));
                    graph.addDataset(GraphManager.getInstance()
                            .graphArrow(new Transform(targetPos, 90), 5, 2, "asdf", Color.yellow));
                    graph.addDataset(GraphManager.getInstance().graphArrow(finalTransform, 5, 2, "asdf", Color.white));

                }
            });
            //transform = transform.add(new Transform(.1,-.1));

            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}