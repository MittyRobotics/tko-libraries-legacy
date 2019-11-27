package com.amhsrobotics.libs.visualization;

import com.amhsrobotics.libs.geometry.Position;
import com.amhsrobotics.libs.geometry.Rotation;
import com.amhsrobotics.libs.geometry.Transform;
import com.amhsrobotics.libs.visualization.graphs.Graph;
import com.amhsrobotics.libs.visualization.themes.DarkTheme;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.swing.*;

public class GraphMain {
	public static void main(String[] args) {
		
		Graph graph = new Graph("Graph", "X", "Y");
		graph.setVisible(true);
		graph.setGraphTheme(new DarkTheme());
		
		Transform transform = new Transform(new Position(15, 5), new Rotation(45));
		Transform origin = new Transform(new Position(10, 5), new Rotation(-45));
		
		XYSeriesCollection dataset = new XYSeriesCollection();
		XYSeries series = new XYSeries("Test");
		
		XYSeries series1 = new XYSeries("Error Point");
		
		XYSeries series2 = new XYSeries("Origin");
		
		
		XYSeries series3 = new XYSeries("Relative origin");
		
		series.add(transform.getPosition().getX(), transform.getPosition().getY());
		series.add(transform.getPosition().getX() + transform.getRotation().cos() * 2, transform.getPosition().getY() + transform.getRotation().sin() * 2);
		
		
		Transform error = transform.relativeTo(origin);
		
		series1.add(error.getPosition().getX(), error.getPosition().getY());
		series1.add(error.getPosition().getX() + error.getRotation().cos() * 2, error.getPosition().getY() + error.getRotation().sin() * 2);
		
		series2.add(origin.getPosition().getX(), origin.getPosition().getY());
		series2.add(origin.getPosition().getX() + origin.getRotation().cos() * 2, origin.getPosition().getY() + origin.getRotation().sin() * 2);
		
		
		series3.add(0, 0);
		series3.add(2, 0);
		
		System.out.println(error.toString());
		
		
		dataset.addSeries(series);
		dataset.addSeries(series1);
		dataset.addSeries(series2);
		dataset.addSeries(series3);
		XYDataset[] datasets = new XYDataset[]{dataset};
		
		graph.setDatasets(datasets);
		
		graph.resizeGraph(-20, 20, -20, 20);
		
		graph.setSize(800,800);
		
	}
	
}
