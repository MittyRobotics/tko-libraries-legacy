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

package com.github.mittyrobotics.visualization.themes;

import java.awt.*;

public class GraphTheme {
	
	private Color graphBackgroundColor;
	private Color gridColor;
	private Color frameBackgroundColor;
	private Color titleColor;
	private Color labelColor;
	private Color tickLabelColor;
	
	public GraphTheme(Color graphBackgroundColor, Color gridColor, Color frameBackgroundColor, Color titleColor, Color labelColor, Color tickLabelColor) {
		this.graphBackgroundColor = graphBackgroundColor;
		this.gridColor = gridColor;
		this.frameBackgroundColor = frameBackgroundColor;
		this.titleColor = titleColor;
		this.labelColor = labelColor;
		this.tickLabelColor = tickLabelColor;
	}
	
	public Color getGraphBackgroundColor() {
		return graphBackgroundColor;
	}
	
	public void setGraphBackgroundColor(Color graphBackgroundColor) {
		this.graphBackgroundColor = graphBackgroundColor;
	}
	
	public Color getGridColor() {
		return gridColor;
	}
	
	public void setGridColor(Color gridColor) {
		this.gridColor = gridColor;
	}
	
	public Color getFrameBackgroundColor() {
		return frameBackgroundColor;
	}
	
	public void setFrameBackgroundColor(Color frameBackgroundColor) {
		this.frameBackgroundColor = frameBackgroundColor;
	}
	
	public Color getTitleColor() {
		return titleColor;
	}
	
	public void setTitleColor(Color titleColor) {
		this.titleColor = titleColor;
	}
	
	public Color getLabelColor() {
		return labelColor;
	}
	
	public void setLabelColor(Color labelColor) {
		this.labelColor = labelColor;
	}
	
	public Color getTickLabelColor() {
		return tickLabelColor;
	}
	
	public void setTickLabelColor(Color tickLabelColor) {
		this.tickLabelColor = tickLabelColor;
	}
}
