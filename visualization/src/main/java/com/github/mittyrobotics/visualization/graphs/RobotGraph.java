package com.github.mittyrobotics.visualization.graphs;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.visualization.util.GraphManager;
import com.github.mittyrobotics.visualization.util.XYSeriesCollectionWithRender;

import java.awt.*;

public class RobotGraph extends Graph {
	
	private static RobotGraph instance = new RobotGraph();
	
	public RobotGraph() {
		super("Robot Graph", "x", "y");
		resizeGraph(-200, 200, -200, 200);
		setSize(800, 800);
	}
	
	public static RobotGraph getInstance() {
		return instance;
	}
	
	public void graphRobot(Transform robotTransform, double width, double length) {
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[]{
				GraphManager.getInstance().graphRectangle(robotTransform, width, length, "robot", Color.white),
				GraphManager.getInstance().graphArrow(robotTransform, length / 2, 1, "robot Transform", Color.white)
		};
		setDatasets(datasets);
	}
}
