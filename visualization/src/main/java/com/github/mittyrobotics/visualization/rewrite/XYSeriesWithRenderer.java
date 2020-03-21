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

package com.github.mittyrobotics.visualization.rewrite;

import org.jfree.data.xy.XYSeries;

import java.awt.*;

public class XYSeriesWithRenderer extends XYSeries {
    private Color color;
    private boolean showLines;
    private boolean showShapes;
    private Shape shape;

    public XYSeriesWithRenderer(XYSeries series) {
        this(series.getKey(),series.getAutoSort(),series.getAllowDuplicateXValues(), null, true, true, null);
        for(int i = 0; i < series.getItemCount(); i++){
            this.add(series.getDataItem(i));
        }
    }

    public XYSeriesWithRenderer(Comparable key) {
        this(key, true, true, null, true, true, null);
    }

    public XYSeriesWithRenderer(Comparable key, boolean autoSort) {
        this(key, autoSort, true, null, true, true, null);
    }

    public XYSeriesWithRenderer(Comparable key, boolean autoSort, boolean allowDuplicateXValues) {
        this(key, autoSort, allowDuplicateXValues, null, true, true, null);
    }

    public XYSeriesWithRenderer(Comparable key, Color color, boolean showLines, boolean showShapes, Shape shape) {
        super(key);
    }

    public XYSeriesWithRenderer(Comparable key, boolean autoSort, Color color, boolean showLines, boolean showShapes,
                                Shape shape) {
        super(key, autoSort);
    }

    public XYSeriesWithRenderer(Comparable key, boolean autoSort, boolean allowDuplicateXValues, Color color,
                                boolean showLines, boolean showShapes, Shape shape) {
        super(key, autoSort, allowDuplicateXValues);
        this.color = color;
        this.showLines = showLines;
        this.showShapes = showShapes;
        this.shape = shape;
    }

    public void setRenderer(Color color, boolean showLines, boolean showShapes, Shape shape){
        this.color = color;
        this.showLines = showLines;
        this.showShapes = showShapes;
        this.shape = shape;
    }

    public Color getColor() {
        return color;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public boolean isShowLines() {
        return showLines;
    }

    public void setShowLines(boolean showLines) {
        this.showLines = showLines;
    }

    public boolean isShowShapes() {
        return showShapes;
    }

    public void setShowShapes(boolean showShapes) {
        this.showShapes = showShapes;
    }

    public Shape getShape() {
        return shape;
    }

    public void setShape(Shape shape) {
        this.shape = shape;
    }
}
