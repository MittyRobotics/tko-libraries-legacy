package com.amhsrobotics.libs.visualization.graphs;

import org.jfree.data.xy.XYSeriesCollection;

import java.awt.*;

public class XYSeriesCollectionWithRender extends XYSeriesCollection {
	private boolean showLines;
	private boolean showPoints;
	private Color color;
	private Shape shape;
	
	public XYSeriesCollectionWithRender(){
		this(true,true, Color.white, new Rectangle(2,2));
	}
	
	public XYSeriesCollectionWithRender(boolean showLines, boolean showPoints, Color color, Shape shape){
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
