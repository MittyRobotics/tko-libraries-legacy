package com.github.mittyrobotics.visualization.graphs;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.visualization.util.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

public class MotorSimGraph extends Graph {
	XYSeries positionSeries;
	XYSeries velocitySeries;
	XYSeries voltageSeries;
	XYSeries setpointSeries;
	XYSeries errorSeries;
	
	public MotorSimGraph() {
		super();
		positionSeries = new XYSeries("Position", false);
		velocitySeries = new XYSeries("Velocity", false);
		voltageSeries = new XYSeries("Voltage", false);
		setpointSeries = new XYSeries("Setpoint", false);
		errorSeries = new XYSeries("Error", false);
		
		final XYSeriesCollectionWithRender data = new XYSeriesCollectionWithRender();
		data.addSeries(positionSeries);
		data.addSeries(velocitySeries);
		data.addSeries(voltageSeries);
		data.addSeries(setpointSeries);
		data.addSeries(errorSeries);
		
		addDataset(data);
		
	}
	
	public void addPosition(double position, double time) {
		//convert to inches
		positionSeries.add(time, position * Conversions.M_TO_IN);
	}
	
	public void addVelocity(double velocity, double time) {
		//convert to inches
		velocitySeries.add(time, velocity * Conversions.M_TO_IN);
	}
	
	public void addVoltage(double voltage, double time) {
		//convert to inches
		voltageSeries.add(time, voltage);
	}
	
	public void addSetpoint(double setpoint, double time) {
		//convert to inches
		setpointSeries.add(time, setpoint * Conversions.M_TO_IN);
	}
	
	public void addError(double error, double time) {
		//convert to inches
		errorSeries.add(time, error);
	}
}
