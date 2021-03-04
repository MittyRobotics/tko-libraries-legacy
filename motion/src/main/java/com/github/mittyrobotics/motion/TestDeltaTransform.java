package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.GraphUtil;
import com.github.mittyrobotics.visualization.XYSeriesWithRenderer;

public class TestDeltaTransform {
    public static void main(String[] args) throws InterruptedException {
        Graph graph = new Graph();
        Transform robotTransform = new Transform();
        graph.scaleGraphToScale(.5, 0, 0);
        DrivetrainState state = DrivetrainState.fromWheelSpeeds(-10, 10, 20);
        while(true){
            robotTransform = robotTransform.add(state.getRotatedVelocityTransform(robotTransform.getRotation()).multiply(0.02));
            System.out.println(state.getRotatedVelocityTransform(robotTransform.getRotation()).multiply(0.02));
            graph.changeSeries("Robot", GraphUtil.populateSeries(new XYSeriesWithRenderer("Robot"), GraphUtil.rectangle(robotTransform, 20, 30)));
            Thread.sleep((long) 20);
        }
    }
}
