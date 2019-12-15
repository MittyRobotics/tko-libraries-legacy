package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datacollection.performance.TimeMonitor;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.datatypes.PathSegment;
import com.github.mittyrobotics.path.generation.datatypes.TransformWithSegment;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphManager;

import java.awt.*;

public class Main {
	public static void main(String[] args) throws InterruptedException {
		Transform[] waypoints = new Transform[]{
				new Transform(0, 0, 0),
				new Transform(100, 100, 0)
		};
		
		TimeMonitor timeMonitor1 = new TimeMonitor("Generate Path");
		timeMonitor1.start();
		CubicHermitePath path = new CubicHermitePath(waypoints, new MotionState(0), new MotionState(0), new VelocityConstraints(5, 5, 20), 8, .2, 10);
		timeMonitor1.end();
		timeMonitor1.printMillis();
		
		
		Graph graph = new Graph("graph", "y", "x");
		
		graph.setSize(800, 800);
		graph.getChart().removeLegend();
		graph.setVisible(true);
		//graph.resizeGraph(-20,100,-20,100);
		
		Transform transform = new Transform(50, 50,0);
		TransformWithSegment closestTransformWithSegment = path.getClosestTransformWithSegment(transform, 0,false);
		PathSegment closestSegment = closestTransformWithSegment.getSegment();
		Transform closestTransform = closestTransformWithSegment.getTransform();
		TransformWithSegment targetTransformWithSegment = path.getClosestTransformWithSegment(transform, 20,false);
		PathSegment targetSegment = targetTransformWithSegment.getSegment();
		Transform targetTransform = targetTransformWithSegment.getTransform();
		Color color = Color.red;
		if (closestSegment.getPathSegmentType() == PathSegmentType.ARC) {
			graph.addDataset(GraphManager.getInstance().graphArc(closestSegment.getArcSegment(), "arcdfg", color));
		} else if (closestSegment.getPathSegmentType() == PathSegmentType.LINE) {
			graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestSegment.getStartPoint(), closestSegment.getStartPoint().angleTo(closestSegment.getEndPoint())), closestSegment.getLineSegment().getFirstPoint().distance(closestSegment.getLineSegment().getSecondPoint()), 0, "arsdfgc" , color));
		}
		
		graph.addDataset(GraphManager.getInstance().graphArrow(closestTransform,5,2,"adsf",Color.white));
		color = Color.white;
		if (targetSegment.getPathSegmentType() == PathSegmentType.ARC) {
			graph.addDataset(GraphManager.getInstance().graphArc(targetSegment.getArcSegment(), "arcdfg", color));
		} else if (targetSegment.getPathSegmentType() == PathSegmentType.LINE) {
			graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(targetSegment.getStartPoint(), targetSegment.getStartPoint().angleTo(targetSegment.getEndPoint())), targetSegment.getLineSegment().getFirstPoint().distance(targetSegment.getLineSegment().getSecondPoint()), 0, "arsdfgc" , color));
		}
		
		graph.addDataset(GraphManager.getInstance().graphArrow(targetTransform,5,2,"adsf",Color.green));
		
		
		double b = path.getSegments().size();
		for (int i = 0; i < b; i++) {
			color = Color.cyan;
			if (path.getSegments().get(i).getPathSegmentType() == PathSegmentType.ARC) {
				graph.addDataset(GraphManager.getInstance().graphArc(path.getSegments().get(i).getArcSegment(), "arc" + i, color));
			} else if (path.getSegments().get(i).getPathSegmentType() == PathSegmentType.LINE) {
				graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(path.getSegments().get(i).getStartPoint(), path.getSegments().get(i).getStartPoint().angleTo(path.getSegments().get(i).getEndPoint())), path.getSegments().get(i).getLineSegment().getFirstPoint().distance(path.getSegments().get(i).getLineSegment().getSecondPoint()), 0, "arc" + i, color));
			}
		}
		

		
	}
}
