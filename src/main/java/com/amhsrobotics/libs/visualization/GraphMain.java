package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.auton.motionprofile.JointVelocityMotionProfile;
import com.amhsrobotics.libs.auton.path.generation.CubicHermitePath;
import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.math.geometry.Position;
import com.amhsrobotics.libs.math.geometry.Rotation;
import com.amhsrobotics.libs.math.geometry.Transform;
import com.amhsrobotics.libs.visualization.graphs.GraphManager;
import com.amhsrobotics.libs.visualization.graphs.RobotSimGraph;
import com.amhsrobotics.libs.visualization.graphs.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import java.awt.*;

public class GraphMain {
	public static void main(String[] args) {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);

		
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true,false, Color.white,null);
		XYSeries series = new XYSeries("Test");
		
		double maxAccel = 40;
		double maxDecel = 40;
		double maxVel = 150;
		
		
		VelocityConstraints velocityConstraints = new VelocityConstraints(maxAccel,maxDecel,maxVel,20,0,0);
		
		Transform[] waypoints = new Transform[]{
				new Transform(0,0),
				new Transform(20,50),
				new Transform(100,50)
		};
		
		Path path = new CubicHermitePath(waypoints,velocityConstraints,20,100);
		

		dataset.addSeries(series);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[1];

		for(double i = 0; i < path.getVelocityProfile().getTotalTime(); i+= 0.01){
			series.add(i,path.getVelocityProfile().getVelocityAtTime(i));
		}
		
		datasets[0] = dataset;
		
		graph.setDatasets(datasets);
		//graph.resizeGraph(-10,110,-10,110);
		graph.getChart().removeLegend();
		graph.setSize(800,800);
		
	}
	
}
