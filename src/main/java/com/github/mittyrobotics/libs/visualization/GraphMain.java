package com.github.mittyrobotics.libs.visualization;

import com.github.mittyrobotics.libs.util.geometry.Transform;
import com.github.mittyrobotics.libs.visualization.graphs.GraphManager;
import com.github.mittyrobotics.libs.visualization.graphs.RobotSimGraph;
import com.github.mittyrobotics.libs.visualization.graphs.XYSeriesCollectionWithRender;

import java.awt.*;

public class GraphMain {
	public static void main(String[] args) throws InterruptedException {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);
		
		//graph.resizeGraph(-10, 120, -10, 120);
		graph.getChart().removeLegend();
		graph.setSize(800, 800);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[3];
		
		Transform transform1 = new Transform(3454,234,33);
		Transform transform2 = new Transform(4,3,10);

		Transform transform = transform2.transformBy(transform1);
		datasets[0] = GraphManager.getInstance().graphArrow(transform.getPosition().getX(),transform.getPosition().getY(),4,1,transform.getRotation().getHeading(),"asdf", Color.white);
		datasets[1] = GraphManager.getInstance().graphArrow(transform1.getPosition().getX(),transform1.getPosition().getY(),4,1,transform1.getRotation().getHeading(),"asdf", Color.red);
		datasets[2] = GraphManager.getInstance().graphArrow(transform2.getPosition().getX(),transform2.getPosition().getY(),4,1,transform2.getRotation().getHeading(),"asdf", Color.blue);
		
		System.out.println(transform.relativeTo(transform2));

		graph.setDatasets(datasets);
		
	}
}
