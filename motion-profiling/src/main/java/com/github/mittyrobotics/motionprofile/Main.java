package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.motionprofile.util.datatypes.MechanismBounds;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.visualization.graphs.Graph;
import com.github.mittyrobotics.visualization.util.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

import java.awt.*;

public class Main {
	public static void main(String[] args) {
		MotionState startFrame = new MotionState(13.559322033898304,5.207556439232954);
		MotionState endFrame = new MotionState(16.94915254237288, 5.822225097395819);
		VelocityConstraints velocityConstraints = new VelocityConstraints(1,1,20);
		MechanismBounds bounds = new MechanismBounds(0,0);
		TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(startFrame,endFrame,velocityConstraints,bounds);
		
		Graph graph = new Graph("Graph","y","x");
		graph.setVisible(true);
		
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true,true,new Color(90, 199, 218),null);
		
		XYSeries series = new XYSeries("Velocity");
		XYSeries series1 = new XYSeries("Position");
		for(double i = 0; i < motionProfile.getTotalTime(); i+= 0.01){
			series.add(i,motionProfile.getFrameAtTime(i).getVelocity());
			series1.add(i,motionProfile.getFrameAtTime(i).getPosition());
		}
		
		dataset.addSeries(series);
		dataset.addSeries(series1);
		graph.setDatasets(new XYSeriesCollectionWithRender[]{dataset});
	}
}
