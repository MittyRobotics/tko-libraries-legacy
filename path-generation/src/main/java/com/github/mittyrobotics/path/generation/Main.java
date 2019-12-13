package com.github.mittyrobotics.path.generation;

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
		
		CubicHermitePath path = new CubicHermitePath(waypoints, new MotionState(0), new MotionState(0), new VelocityConstraints(5, 5, 20), 20,.2,10);
		
		Graph graph = new Graph("graph", "y", "x");
		
		graph.setSize(800, 800);
		graph.getChart().removeLegend();
		graph.setVisible(true);
		//graph.resizeGraph(-20,100,-20,100);
		double prevA = 0;
		double b = path.getSegments().size();
		

		for (int i = 0; i < b; i++) {
			Color color = Color.cyan;
			
			if (path.getSegments().get(i).getPathSegmentType() == PathSegmentType.ARC) {
				graph.addDataset(GraphManager.getInstance().graphArc(path.getSegments().get(i).getArcSegment(), "arc" + i, color));
			} else if (path.getSegments().get(i).getPathSegmentType() == PathSegmentType.LINE) {
				graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(path.getSegments().get(i).getStartPoint(), path.getSegments().get(i).getStartPoint().angleTo(path.getSegments().get(i).getEndPoint())), path.getSegments().get(i).getLineSegment().getFirstPoint().distance(path.getSegments().get(i).getLineSegment().getSecondPoint()), 0, "arc" + i,color));
			}
			XYSeries series = new XYSeries("Motion Profile" + i);
			TrapezoidalMotionProfile motionProfile = path.getSegments().get(i).getVelocityMotionProfile();
			//System.out.println(motionProfile.getStartMotionState() + " " + motionProfile.getEndMotionState() + " " + path.getSegments().get(i).getSegmentDistance() + " " + path.getSegments().get(i).getMaxVelocity());
			
			for (double a = 0; a < motionProfile.getTotalTime(); a += 0.01) {
				//System.out.println(a+prevA + " " + motionProfile.getFrameAtTime(a).getPosition() + " " + motionProfile.getFrameAtTime(a).getVelocity());
				//series.add(prevA + a, motionProfile.getFrameAtTime(a).getVelocity());
			}
			
			prevA += motionProfile.getTotalTime();
			
			XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, Color.cyan, null);
			dataset.addSeries(series);
			graph.addDataset(dataset);
			// Thread.sleep(2000);
		}
		
		Position pos = new Position(-40,0);
		PathSegment closestSegment = path.getClosestSegment(pos,0);
		PathSegment lookaheadSegment = path.getClosestSegment(closestSegment.getClosestPointOnSegment(pos),20);
		
		System.out.println(lookaheadSegment.getClosestPointOnSegment(closestSegment.getClosestPointOnSegment(pos),20));
		
		graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestSegment.getClosestPointOnSegment(pos)),5,2,"asdf",Color.green));
		graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(lookaheadSegment.getClosestPointOnSegment(closestSegment.getClosestPointOnSegment(pos),20)),5,2,"asdf",Color.white));
		
		if (closestSegment.getPathSegmentType() == PathSegmentType.ARC) {
			graph.addDataset(GraphManager.getInstance().graphArc(closestSegment.getArcSegment(), "arcg" , Color.blue));
		} else if (closestSegment.getPathSegmentType() == PathSegmentType.LINE) {
			graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(closestSegment.getStartPoint(), closestSegment.getStartPoint().angleTo(closestSegment.getEndPoint())), closestSegment.getLineSegment().getFirstPoint().distance(closestSegment.getLineSegment().getSecondPoint()), 0, "arcasdf" ,Color.blue));
		}
		
		if (lookaheadSegment.getPathSegmentType() == PathSegmentType.ARC) {
			graph.addDataset(GraphManager.getInstance().graphArc(lookaheadSegment.getArcSegment(), "ardcg" , Color.red));
		} else if (lookaheadSegment.getPathSegmentType() == PathSegmentType.LINE) {
			graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(lookaheadSegment.getStartPoint(), lookaheadSegment.getStartPoint().angleTo(lookaheadSegment.getEndPoint())), lookaheadSegment.getLineSegment().getFirstPoint().distance(lookaheadSegment.getLineSegment().getSecondPoint()), 0, "arcadsdf" ,Color.red));
		}
	}
}
