package com.github.mittyrobotics.path.generation;

import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.path.generation.paths.CubicHermitePath;
import com.github.mittyrobotics.path.generation.util.enums.PathSegmentType;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphManager;
import org.jfree.chart.axis.SymbolAxis;

import java.awt.*;

public class Main {
    public static void main(String[] args) {
        Transform[] waypoints = new Transform[]{
                new Transform(0,0,0),
                new Transform(10,100,0)
        };

        CubicHermitePath path = new CubicHermitePath(waypoints,new MotionState(0),new MotionState(0),new VelocityConstraints(10,10,50),10);

        Graph graph = new Graph("graph","y","x");

        graph.setSize(800,800);
        graph.resizeGraph(-20,100,-20,100);

        for(int i = 0; i < path.getSegments().size(); i++){
            if(path.getSegments().get(i).getPathSegmentType() == PathSegmentType.ARC){
                graph.addDataset(GraphManager.getInstance().graphArc(path.getSegments().get(i).getArcSegment(),"arc" + i, Color.CYAN));
            }
            else if(path.getSegments().get(i).getPathSegmentType() == PathSegmentType.LINE){
                graph.addDataset(GraphManager.getInstance().graphArrow(new Transform(path.getSegments().get(i).getStartPoint(),path.getSegments().get(i).getStartPoint().angleTo(path.getSegments().get(i).getEndPoint())), path.getSegments().get(i).getLineSegment().getFirstPoint().distance(path.getSegments().get(i).getLineSegment().getSecondPoint()), 0,"arc" + i, Color.CYAN));
            }
        }

        graph.setVisible(true);

    }
}
