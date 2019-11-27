package com.amhsrobotics.libs.visualization.themes;

import java.awt.*;

public class GraphTheme {
	
	private Color graphBackgroundColor;
	private Color gridColor;
	private Color frameBackgroundColor;
	private Color titleColor;
	private Color labelColor;
	private Color tickLabelColor;
	
	public GraphTheme(Color graphBackgroundColor, Color gridColor, Color frameBackgroundColor, Color titleColor, Color labelColor, Color tickLabelColor){
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
