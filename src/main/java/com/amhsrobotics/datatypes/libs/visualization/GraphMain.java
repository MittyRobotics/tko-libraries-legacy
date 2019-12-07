package com.amhsrobotics.datatypes.libs.visualization;

import com.amhsrobotics.datatypes.libs.visualization.graphs.RobotSimGraph;
import com.amhsrobotics.datatypes.libs.visualization.graphs.XYSeriesCollectionWithRender;

public class GraphMain {
	public static void main(String[] args) throws InterruptedException {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);
		
		//graph.resizeGraph(-10, 120, -10, 120);
		graph.getChart().removeLegend();
		graph.setSize(800, 800);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[0];
		


		
		graph.setDatasets(datasets);
		
	}
}
