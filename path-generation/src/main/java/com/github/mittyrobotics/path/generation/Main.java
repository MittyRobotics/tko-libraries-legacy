package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.enums.RoundMode;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motionprofile.TrapezoidalMotionProfile;
import com.github.mittyrobotics.path.generation.datatypes.PathSegment;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.enums.PathSegmentType;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphManager;
import com.github.mittyrobotics.visualization.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

import java.awt.*;

public class Main {
	public static void main(String[] args) throws InterruptedException {
		Transform[] waypoints = new Transform[]{
				new Transform(0, 0, 0),
				new Transform(100, 100, 0)
		};
		
		CubicHermitePath path = new CubicHermitePath(waypoints, new MotionState(0), new MotionState(0), new VelocityConstraints(5, 5, 20), 20, .2, 10);
		
		Graph graph = new Graph("graph", "y", "x");
		
		graph.setSize(800, 800);
		graph.getChart().removeLegend();
		graph.setVisible(true);
		//graph.resizeGraph(-20,100,-20,100);
		
		double b = path.getSegments().size();
		for (int i = 0; i < b; i++) {
			Color color = Color.cyan;
			if (path.getSegments().get(i).getPathSegmentType() == PathSegmentType.ARC) {
				graph.addDataset(GraphManager.getInstance().graphArc(path.getSegments().get(i).getArcSegment(), "arc" + i, color));
			} else if (path.getSegments().get(i).getPathSegmentType() == PathSegmentType.LINE) {
				graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(path.getSegments().get(i).getStartPoint(), path.getSegments().get(i).getStartPoint().angleTo(path.getSegments().get(i).getEndPoint())), path.getSegments().get(i).getLineSegment().getFirstPoint().distance(path.getSegments().get(i).getLineSegment().getSecondPoint()), 0, "arc" + i, color));
			}
		}
		
		Position pos = new Position(-250, 0);
		PathSegment closestSegment = path.getClosestSegment(pos, 0);
		Position closestPosition = closestSegment.getClosestPointOnSegment(pos).get();
		PathSegment lookaheadSegment = path.getClosestSegment(closestPosition, 20);
		graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestPosition), 5, 2, "asdf", Color.green));
		graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(lookaheadSegment.getClosestPointOnSegment(closestPosition, 20, RoundMode.ROUND_UP).get()), 5, 2, "asdf", Color.white));
		
	}
}
