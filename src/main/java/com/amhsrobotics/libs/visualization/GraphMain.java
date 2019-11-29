package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.auton.motionprofile.JointVelocityMotionProfile;
import com.amhsrobotics.libs.auton.path.follow.AutonDriver;
import com.amhsrobotics.libs.auton.path.follow.PurePursuitController;
import com.amhsrobotics.libs.auton.path.generation.CubicHermitePath;
import com.amhsrobotics.libs.auton.path.generation.Path;
import com.amhsrobotics.libs.datatypes.DrivetrainWheelVelocities;
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
	public static void main(String[] args) throws InterruptedException {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);
		
		graph.resizeGraph(-10,110,-10,110);
		graph.getChart().removeLegend();
		graph.setSize(800,800);
		AutonDriver.getInstance().updateRobotState(new Transform(), new DrivetrainWheelVelocities(0,0));
		while(true){
			XYSeriesCollectionWithRender pathDataset = new XYSeriesCollectionWithRender(true,false, Color.white,null);
			XYSeriesCollectionWithRender pointDataset = new XYSeriesCollectionWithRender(false,true, Color.BLUE,new Rectangle(2,2));
			
			XYSeriesCollectionWithRender lookaheadDataset = new XYSeriesCollectionWithRender(false,true, Color.RED,new Rectangle(2,2));
			
			XYSeries series = new XYSeries("Path");
			XYSeries series1 = new XYSeries("Point");
			XYSeries series2 = new XYSeries("LookaheadPoint");
Path path = new CubicHermitePath(new Transform[]{new Transform(0,0), new Transform(100,50)},new VelocityConstraints(20,20,100,20,0,0),20,10);
			
			AutonDriver.getInstance().updateRobotState(new Transform(AutonDriver.getInstance().getRobotTransform().getPosition().getX() + .1,AutonDriver.getInstance().getRobotTransform().getPosition().getY() + 0.05,45/2), new DrivetrainWheelVelocities(0,0));
			
			
			
			for(int i = 0; i < path.getTrajectoryPoints().length; i++){
				series.add(path.getTrajectoryPoints()[i].getPosition().getX(),path.getTrajectoryPoints()[i].getPosition().getY());
			}
			lookaheadDataset.addSeries(series2);
			
			pathDataset.addSeries(series);
			pointDataset.addSeries(series1);
			
			
			XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[]{
					GraphManager.getInstance().graphArrow(AutonDriver.getInstance().getRobotTransform().getPosition().getX(),AutonDriver.getInstance().getRobotTransform().getPosition().getY(),5,2,AutonDriver.getInstance().getRobotTransform().getRotation().getHeading(), "Robot"),
					pathDataset,
					pointDataset,
					lookaheadDataset
			};
			
			PurePursuitController controller = new PurePursuitController(path);
			controller.update();
			
			series1.add(controller.getActualClosestPoint().getPosition().getX(),controller.getActualClosestPoint().getPosition().getY());
			//series1.add(controller.getRoughClosestPoint().getPosition().getX(), controller.getRoughClosestPoint().getPosition().getY());
			//series2.add(controller.getRoughTargetPoint().getPosition().getX(),controller.getRoughTargetPoint().getPosition().getY());
			series2.add(controller.getActualTargetPoint().getPosition().getX(),controller.getActualTargetPoint().getPosition().getY());
			graph.setDatasets(datasets);

			Thread.sleep(20);
		}
	}
}
