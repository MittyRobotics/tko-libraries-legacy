package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.auton.path.follow.PurePursuitController;
import com.amhsrobotics.libs.auton.path.generation.CubicHermitePath;
import com.amhsrobotics.libs.auton.path.generation.Path;
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
		
		graph.resizeGraph(-10,120,-10,120);
		graph.getChart().removeLegend();
		graph.setSize(800,800);
		
		Transform[] waypoints = new Transform[]{
				new Transform(0,0),
				new Transform(50,25,0),
				new Transform(100,50,0),
				new Transform(110,50,0)
		};
		
		
		Path path = new CubicHermitePath(waypoints,null,10);
		
		XYSeriesCollectionWithRender[] datasets = new XYSeriesCollectionWithRender[path.getSegments().size()+2];
		
		PurePursuitController controller = new PurePursuitController(path);
		Transform robotTransform = new Transform();
		while(true){
			
			robotTransform = robotTransform.transformBy(new Transform(0.1,0.05));
			
			
			
			datasets[0] = GraphManager.getInstance().graphArrow(robotTransform.getPosition().getX(),robotTransform.getPosition().getY(),5,2,robotTransform.getRotation().getHeading(),"Robot",Color.WHITE);
			
			XYSeriesCollectionWithRender closestPointDataset = new XYSeriesCollectionWithRender();
			
			XYSeries closestPointSeries = new XYSeries("Closest Point");
			
			
			closestPointDataset.addSeries(closestPointSeries);
			
			datasets[1] = closestPointDataset;
			
			
			
			PathSegment closestSegment = controller.getClosestPathSegment(robotTransform);
			
			Position closestPoint = controller.getClosestSegmentPoint(robotTransform);
			
			closestPointSeries.add(closestPoint.getX(),closestPoint.getY());
			
			for(int i = 0; i < path.getSegments().size(); i++){
				
				if(path.getSegments().get(i).getType() == PathSegmentType.ARC){
					Arc arc = path.getSegments().get(i).getArcSegment().getArc();
					Color color = Color.red;
					if(closestSegment.getStartPoint().getPosition().distance(path.getSegments().get(i).getStartPoint().getPosition()) < 2e-16){
						color = Color.blue;
					}
					datasets[i+2] = GraphManager.getInstance().graphArc(arc.getCenter().getX(),arc.getCenter().getY(),arc.getRadius(),arc.getMinAngle(),arc.getMaxAngle(), "Arc"+ i, color);
				}
				else{
					Line line = path.getSegments().get(i).getLineSegment().getLine();
					Color color = Color.red;
					if(closestSegment.getStartPoint().getPosition().distance(path.getSegments().get(i).getStartPoint().getPosition()) < 2e-16){
						color = Color.blue;
					}
					datasets[i+2] = GraphManager.getInstance().graphArrow(line.getP1().getX(),line.getP1().getY(),line.getP1().distance(line.getP2()),2,line.getTransform().getRotation().getHeading(), "Line"+i,color);
				}
			}
			graph.setDatasets(datasets);
			Thread.sleep(20);
		}
	}
}
