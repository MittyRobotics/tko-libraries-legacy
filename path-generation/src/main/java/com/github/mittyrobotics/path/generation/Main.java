package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.splines.CubicHermiteSpline;
import com.github.mittyrobotics.visualization.graphs.Graph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import java.awt.*;

public class Main {
	public static void main(String[] args) {
		CubicHermiteSpline spline = new CubicHermiteSpline(new Transform(0, 0, 0), new Transform(100, 24, 0));
		
		Graph graph = new Graph();
		
		//graph.resizeGraph(-20, 120, -20, 120);
		
		graph.getChart().removeLegend();
		CubicHermitePath path = new CubicHermitePath(new Transform[]{
				new Transform(-100, 50, 0),
				new Transform(100, 0, 90)
		});
		Transform transform = new Transform(-50, 50);
		graph.clearGraph();
		Position closestPos = path.getClosestTransform(transform.getPosition(), 0, false, 10, 1000).getPosition();
		Position targetPos = path.getClosestTransform(closestPos, 20, false, 10, 1000).getPosition();
		graph.addDataset(GraphManager.getInstance().graphParametric(path,0.01,2, 1, "spline", Color.cyan));
		graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestPos, 90), 5, 2, "asdf", Color.green));
		graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(targetPos, 90), 5, 2, "asdf", Color.yellow));
		graph.addDataset(GraphManager.getInstance().graphArrow(transform, 5, 2, "asdf", Color.white));
		
		try {
			Thread.sleep(30);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}