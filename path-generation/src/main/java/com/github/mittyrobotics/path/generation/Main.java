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
		
		graph.resizeGraph(-10,110,-10,110);
		
		graph.getChart().removeLegend();
		CubicHermitePath path = new CubicHermitePath(new Transform[]{
				new Transform(0, 0, 0),
				new Transform(100, 24, 0)
		});
		Transform transform = new Transform(0,0);
		while(true){
			graph.clearGraph();
			graph.addDataset(GraphManager.getInstance().graphParametric(path.getParametrics(), 2, 1, "spline", Color.cyan));
			Position closestPos = path.getClosestPoint(transform.getPosition(),0,false,3);
			graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestPos,90),5,2,"asdf",Color.green));
			
			transform.setPosition(transform.getPosition().add(new Position(.5,0)));
			
			graph.addDataset(GraphManager.getInstance().graphArrow(transform,5,2,"asdf",Color.white));
			
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
