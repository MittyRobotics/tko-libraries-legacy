package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.datatypes.PathTransform;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.visualization.graphs.Graph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import javax.swing.*;
import java.awt.*;

public class Main {
	public static void main(String[] args) {
		Graph graph = new Graph();
		
		graph.resizeGraph(-20, 120, -20, 120);
		
		graph.getChart().removeLegend();
		CubicHermitePath path = new CubicHermitePath(new Transform[]{
				new Transform(0, 0, 0),
				new Transform(100, 100, 0)
		});
		Transform transform = new Transform(0, 0);
		while (true) {
			
			PathTransform closestPos = path.getClosestTransform(transform.getPosition(), 10, 3);
			Position targetPos = path.getClosestTransform(closestPos.getPosition(), 20, false, 10, 3).getPosition();
			
			Transform finalTransform = transform;
			SwingUtilities.invokeLater(new Runnable() {
				@Override
				public void run() {
					graph.clearGraph();
					graph.addDataset(GraphManager.getInstance().graphParametric(path, 0.01, 2, 1, "spline", Color.cyan));
					graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestPos.getPosition(), 90), 5, 2, "asdf", Color.green));
					graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(targetPos, 90), 5, 2, "asdf", Color.yellow));
					graph.addDataset(GraphManager.getInstance().graphArrow(finalTransform, 5, 2, "asdf", Color.white));
					
				}
			});
			//transform = transform.add(new Transform(.1,-.1));
			
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
	}
}