package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;
import com.github.mittyrobotics.motionprofile.util.datatypes.MechanismBounds;
import com.github.mittyrobotics.datatypes.motion.MotionState;
import com.github.mittyrobotics.visualization.Graph;
import com.github.mittyrobotics.visualization.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

import java.awt.*;

public class Main {
	public static void main(String[] args) {
		MotionState startFrame = new MotionState(0,0);
		MotionState endFrame = new MotionState(6.896551724137931,8.304547985373997);
		VelocityConstraints velocityConstraints = new VelocityConstraints(5,5,20);
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
