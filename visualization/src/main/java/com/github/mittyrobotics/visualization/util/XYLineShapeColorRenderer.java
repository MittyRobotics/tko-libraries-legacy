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

package com.github.mittyrobotics.visualization.util;

import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;

import java.awt.*;

public class XYLineShapeColorRenderer extends XYLineAndShapeRenderer {

    private final boolean showShapes;
    private final boolean showLines;
    private final Color color;

    public XYLineShapeColorRenderer(boolean showShapes, boolean showLines, Color color) {

        this.showShapes = showShapes;
        this.showLines = showLines;
        this.color = color;
    }

    @Override
    public boolean getItemShapeVisible(int series, int item) {
        return showShapes;
    }

    @Override
    public boolean getItemLineVisible(int series, int item) {
        return showLines;
    }

    @Override
    public Paint getItemPaint(int row, int column) {
        return color;
    }
}
