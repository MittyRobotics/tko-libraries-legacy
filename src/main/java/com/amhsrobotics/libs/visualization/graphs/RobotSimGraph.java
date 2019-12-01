package com.amhsrobotics.libs.visualization.graphs;

import org.jfree.data.xy.XYSeriesCollection;

import java.awt.*;

public class RobotSimGraph extends Graph {
	XYSeriesCollectionWithRender robotDataset;
	XYSeriesCollectionWithRender robotArrowDataset;
	
	public RobotSimGraph(String titleName, String yAxisName, String xAxisName) {
		super("Graph", "Y position (in)", "X position (in)");
	}
	
	public void graphRobot(double x, double y, double heading, double width, double length){
		robotDataset = GraphManager.getInstance().graphRectangle(x,y,width,length,heading,"Robot");
		robotArrowDataset = GraphManager.getInstance().graphArrow(x,y,length/2.5, 2, heading,"Robot Arrow", Color.white);
		setDatasets(new XYSeriesCollectionWithRender[]{robotDataset,robotArrowDataset});
	}
}
