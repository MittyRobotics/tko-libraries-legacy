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

import org.jfree.data.xy.XYSeriesCollection;

import java.awt.*;

public class XYSeriesCollectionWithRender extends XYSeriesCollection {
	private boolean showLines;
	private boolean showPoints;
	private Color color;
	private Shape shape;
	
	public XYSeriesCollectionWithRender() {
		this(true, true, Color.white, new Rectangle(2, 2));
	}
	
	public XYSeriesCollectionWithRender(boolean showLines, boolean showPoints, Color color, Shape shape) {
		super();
		this.showLines = showLines;
		this.showPoints = showPoints;
		this.color = color;
		this.shape = shape;
	}
	
	
	public boolean isShowLines() {
		return showLines;
	}
	
	public void setShowLines(boolean showLines) {
		this.showLines = showLines;
	}
	
	public boolean isShowPoints() {
		return showPoints;
	}
	
	public void setShowPoints(boolean showPoints) {
		this.showPoints = showPoints;
	}
	
	public Color getColor() {
		return color;
	}
	
	public void setColor(Color color) {
		this.color = color;
	}
	
	public Shape getShape() {
		return shape;
	}
	
	public void setShape(Shape shape) {
		this.shape = shape;
	}
}
