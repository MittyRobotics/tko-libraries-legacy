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
