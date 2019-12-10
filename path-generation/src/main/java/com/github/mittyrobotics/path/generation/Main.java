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

        CubicHermitePath path = new CubicHermitePath(waypoints,new MotionState(0),new MotionState(0),new VelocityConstraints(10,10,50),100);

        Graph graph = new Graph("graph","y","x");

        System.out.println( path.getSegments().size());

        for(int i = 0; i < path.getSegments().size(); i++){
            if(path.getSegments().get(i).getPathSegmentType() == PathSegmentType.ARC){
                System.out.println("asdf");
                graph.addDataset(GraphManager.getInstance().graphArc(path.getSegments().get(i).getArcSegment(),"arc" + i, Color.CYAN));
            }
            else if(path.getSegments().get(i).getPathSegmentType() == PathSegmentType.LINE){
                System.out.println("asdf");
                graph.addDataset(GraphManager.getInstance().graphArrow(path.getSegments().get(i).getLineSegment().getFirstPoint().getX(),
                        path.getSegments().get(i).getLineSegment().getFirstPoint().getY(), path.getSegments().get(i).getLineSegment().getFirstPoint().distance(path.getSegments().get(i).getLineSegment().getSecondPoint()),0,path.getSegments().get(i).getLineSegment().getFirstPoint().angleTo(path.getSegments().get(i).getLineSegment().getSecondPoint()).getHeading(),"arc" + i, Color.CYAN));
            }
        }

        graph.setVisible(true);

    }
}
