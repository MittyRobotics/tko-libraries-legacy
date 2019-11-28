package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.auton.motionprofile.JointVelocityMotionProfile;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.math.geometry.Position;
import com.amhsrobotics.libs.math.geometry.Rotation;
import com.amhsrobotics.libs.math.geometry.Transform;
import com.amhsrobotics.libs.visualization.graphs.RobotSimGraph;
import com.amhsrobotics.libs.visualization.graphs.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import java.awt.*;

public class GraphMain {
	public static void main(String[] args) {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);

		Transform transform = new Transform(new Position(15, 5), new Rotation(10));
		Transform origin = new Transform(new Position(10, 5), new Rotation(-45));
		
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true,false, Color.white,null);
		XYSeries series = new XYSeries("Test");
		
		XYSeries series1 = new XYSeries("Error Point");
		
		XYSeries series2 = new XYSeries("Origin");
		
		XYSeries series3 = new XYSeries("Relative origin");
//
//		series.add(transform.getPosition().getX(), transform.getPosition().getY());
//		series.add(transform.getPosition().getX() + transform.getRotation().cos() * 2, transform.getPosition().getY() + transform.getRotation().sin() * 2);
		
		
		Transform error = transform.relativeTo(origin);
		
		series1.add(error.getPosition().getX(), error.getPosition().getY());
		series1.add(error.getPosition().getX() + error.getRotation().cos() * 2, error.getPosition().getY() + error.getRotation().sin() * 2);
		
		series2.add(origin.getPosition().getX(), origin.getPosition().getY());
		series2.add(origin.getPosition().getX() + origin.getRotation().cos() * 2, origin.getPosition().getY() + origin.getRotation().sin() * 2);
//
//		series3.add(0, 0);
//		series3.add(2, 0);
//
		
		double maxAccel = 5;
		double maxDecel = 5;
		double maxVel = 20;
		
		JointVelocityMotionProfile.MotionSegment[] motionSegments = new JointVelocityMotionProfile.MotionSegment[]{
				new JointVelocityMotionProfile.TrapezoidalMotionSegment(0,10,new VelocityConstraints(maxAccel,maxDecel,maxVel,0,10)),
				new JointVelocityMotionProfile.FlatMotionSegment(10,10,20),
				new JointVelocityMotionProfile.TrapezoidalMotionSegment(20,30,new VelocityConstraints(maxAccel,maxDecel,maxVel,10,0)),
		};
		
		
		JointVelocityMotionProfile jointVelocityMotionProfile = new JointVelocityMotionProfile(motionSegments, new VelocityConstraints(maxAccel,maxDecel,maxVel,0,0));
		
		for(double i = 0; i < jointVelocityMotionProfile.getTotalTime()+0.1; i+= .1){
		
			series.add(i,jointVelocityMotionProfile.getVelocityAtTime(i));
		}
		
		System.out.println(transform.toString());
		System.out.println(error.toString());

		dataset.addSeries(series);
		//dataset.addSeries(series1);
		//dataset.addSeries(series2);
		//dataset.addSeries(series3);
		
//		graph.graphRobot(5,5,65,5,10);
//		graph.resizeGraph(-20, 20, -20, 20);
//
		
		graph.setDatasets(new XYSeriesCollectionWithRender[]{dataset});
		graph.setSize(800,800);
		
	}
	
}
