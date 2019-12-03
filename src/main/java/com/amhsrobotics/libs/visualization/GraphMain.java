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
import com.amhsrobotics.libs.util.path.ArcPathSegment;
import com.amhsrobotics.libs.util.path.LinePathSegment;
import com.amhsrobotics.libs.util.path.PathSegment;
import com.amhsrobotics.libs.util.path.PathSegmentType;
import com.amhsrobotics.libs.visualization.graphs.GraphManager;
import com.amhsrobotics.libs.visualization.graphs.RobotSimGraph;
import com.amhsrobotics.libs.visualization.graphs.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

import java.awt.*;
import java.util.Random;

public class GraphMain {
	public static void main(String[] args) throws InterruptedException {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);
		
		//graph.resizeGraph(-10, 120, -10, 120);
		graph.getChart().removeLegend();
		graph.setSize(800, 800);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[0];
		


		
		graph.setDatasets(datasets);
		
	}
}
