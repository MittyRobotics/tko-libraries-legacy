package com.github.mittyrobotics.path.generation;

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
		
		graph.getChart().removeLegend();
		CubicHermitePath path = new CubicHermitePath(new Transform[]{
				new Transform(0, 0, 0),
				new Transform(100, 24, 0),
				new Transform(200, 24, 0)
		});
		
		graph.addDataset(GraphManager.getInstance().graphParametric(path.getParametrics(), 2, 1, "spline", Color.cyan));
	}
}
