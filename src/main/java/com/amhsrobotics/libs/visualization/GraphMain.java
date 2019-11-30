package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.auton.path.follow.AutonDriver;
import com.amhsrobotics.libs.auton.path.follow.rewrite.PurePursuitController;
import com.amhsrobotics.libs.auton.path.generation.rewrite.CubicHermitePath;
import com.amhsrobotics.libs.auton.path.generation.rewrite.Path;
import com.amhsrobotics.libs.datatypes.DrivetrainWheelVelocities;
import com.amhsrobotics.libs.datatypes.VelocityConstraints;
import com.amhsrobotics.libs.util.geometry.Arc;
import com.amhsrobotics.libs.util.geometry.Line;
import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Transform;
import com.amhsrobotics.libs.util.path.PathSegment;
import com.amhsrobotics.libs.util.path.PathSegmentType;
import com.amhsrobotics.libs.visualization.graphs.Graph;
import com.amhsrobotics.libs.visualization.graphs.GraphManager;
import com.amhsrobotics.libs.visualization.graphs.RobotSimGraph;
import com.amhsrobotics.libs.visualization.graphs.XYSeriesCollectionWithRender;
import jdk.tools.jlink.internal.JarArchive;
import org.jfree.data.xy.XYSeries;

import java.awt.*;

public class GraphMain {
	public static void main(String[] args) throws InterruptedException {
		
		RobotSimGraph graph = new RobotSimGraph("Graph", "X", "Y");
		graph.setVisible(true);
		
		graph.resizeGraph(-10,110,-10,110);
		graph.getChart().removeLegend();
		graph.setSize(800,800);
		
		Transform[] waypoints = new Transform[]{
				new Transform(0,0),
				new Transform(50,25,0),
				new Transform(100,50,0)
		};
		
		
		Path path = new CubicHermitePath(waypoints,null,10);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[path.getSegments().size()+1];
		
		PurePursuitController controller = new PurePursuitController(path);
		
		Transform robotTransform = new Transform(50,25,0);
		
		datasets[0] = GraphManager.getInstance().graphArrow(robotTransform.getPosition().getX(),robotTransform.getPosition().getY(),5,2,robotTransform.getRotation().getHeading(),"Robot",Color.WHITE);
		
		PathSegment closestSegment = controller.getClosestPathSegment(robotTransform);
		
		for(int i = 0; i < path.getSegments().size(); i++){
			
			if(path.getSegments().get(i).getType() == PathSegmentType.ARC){
				Arc arc = path.getSegments().get(i).getArcSegment().getArc();
				Color color = Color.red;
				if(closestSegment.getStartPoint().getPosition().distance(path.getSegments().get(i).getStartPoint().getPosition()) < 2e-16){
					color = Color.blue;
				}
				datasets[i+1] = GraphManager.getInstance().graphArc(arc.getCenter().getX(),arc.getCenter().getY(),arc.getRadius(),arc.getMinAngle(),arc.getMaxAngle(), "Arc"+ i, color);
			}
			else{
				Line line = path.getSegments().get(i).getLineSegment().getLine();
				Color color = Color.red;
				if(closestSegment.getStartPoint().getPosition().distance(path.getSegments().get(i).getStartPoint().getPosition()) < 2e-16){
					color = Color.blue;
				}
				datasets[i+1] = GraphManager.getInstance().graphArrow(line.getP1().getX(),line.getP1().getY(),line.getP1().distance(line.getP2()),2,line.getTransform().getRotation().getHeading(), "Line"+i,color);
			}

		}
		
		
		graph.setDatasets(datasets);
	}
}
