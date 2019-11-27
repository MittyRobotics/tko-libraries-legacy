package com.amhsrobotics.libs.visualization.graphs;

import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import sun.jvm.hotspot.debugger.win32.coff.COFFException;

import java.awt.*;

public class CustomRenderer extends XYLineAndShapeRenderer {
	
	private final boolean showShapes;
	private final boolean showLines;
	private final Color color;
	
	public CustomRenderer(boolean showShapes, boolean showLines, Color color){
		
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
