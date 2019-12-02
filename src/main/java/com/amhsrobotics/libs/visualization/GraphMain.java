package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.auton.motionprofile.TrapezoidalMotionProfile;
import com.amhsrobotics.libs.auton.path.follow.PurePursuitController;
import com.amhsrobotics.libs.auton.path.generation.CubicHermitePath;
import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.Arc;
import com.amhsrobotics.libs.util.geometry.Line;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;
import com.amhsrobotics.libs.util.path.PathSegmentType;
import com.amhsrobotics.libs.visualization.graphs.GraphManager;
import com.amhsrobotics.libs.visualization.graphs.RobotSimGraph;
import com.amhsrobotics.libs.visualization.graphs.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

import java.awt.*;

public class GraphMain {
	public static void main(String[] args) throws InterruptedException {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);
		
		//graph.resizeGraph(-10, 120, -10, 120);
		graph.getChart().removeLegend();
		graph.setSize(800, 800);
		
		Transform[] waypoints = new Transform[]{
				new Transform(0, 0),
				new Transform(50, 25, 0),
				new Transform(100, 50, 0),
				new Transform(110, 50, 0)
		};
		
		
		Path path = new CubicHermitePath(waypoints, new VelocityConstraints(20,5,20,10,0,0), 10);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[1];
		
		XYSeriesCollectionWithRender velocityDataset = new XYSeriesCollectionWithRender();
		
		XYSeries series = new XYSeries("asdfasdf");
		double prevTime = 0;
		for (int i = 0; i <path.getSegments().size(); i++) {
			TrapezoidalMotionProfile motionProfile = path.getSegments().get(i).getMotionProfile();
			double time = 0;
			System.out.println(i + " " + motionProfile.getMaxVelocity() + " " + path.getSegments().get(i).getSegmentDistance() + " " + motionProfile.getStartVelocity() + " " + motionProfile.getEndVelocity() );
			for(double a = 0; a < motionProfile.gettTotal(); a+=0.01){
				series.add(a+prevTime,motionProfile.getVelocityAtTime(a));
				time = a;
			}
			prevTime += time;
		}

		velocityDataset.addSeries(series);
		
		datasets[0] = velocityDataset;
		graph.setDatasets(datasets);
		
	}
}
