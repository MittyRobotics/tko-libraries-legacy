package com.github.mittyrobotics.visualization.graphs;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.visualization.util.GraphManager;
import com.github.mittyrobotics.visualization.util.XYLineShapeColorRenderer;
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
		getPlot().setDataset(0,datasets[0]);
		getPlot().setDataset(1,datasets[1]);
	}
	
	private int lastIndex = 3;
	@Override
	public void addDataset(XYSeriesCollectionWithRender dataset) {
		getPlot().setDataset(lastIndex, dataset);
		getPlot().setRenderer(lastIndex, new XYLineShapeColorRenderer(dataset.isShowPoints(), dataset.isShowLines(), dataset.getColor()));
		lastIndex++;
	}
	
	public void addPath(XYSeriesCollectionWithRender dataset){
		getPlot().setDataset(2, dataset);
		getPlot().setRenderer(2, new XYLineShapeColorRenderer(dataset.isShowPoints(), dataset.isShowLines(), dataset.getColor()));
	}
	
	@Override
	public void clearGraph() {
		for (int i =3; i < getPlot().getDatasetCount(); i++) {
			getPlot().setDataset(3, null);
		}
		lastIndex = 3;
	}
}
