package com.github.mittyrobotics.visualization;

import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.visualization.graphs.Graph;
import com.github.mittyrobotics.visualization.util.GraphManager;

import java.awt.*;

public class Main {
	public static void main(String[] args) {
		Graph graph = new Graph();
		graph.setVisible(true);
		graph.addDataset(GraphManager.getInstance().graphRectangle(new Transform(5,5,34),20,30,"adsf", Color.white));
	}
}
