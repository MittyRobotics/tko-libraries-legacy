package com.github.mittyrobotics.visualization.util;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.path.Parametric;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import java.awt.*;

public class GraphManager {
	
	private static GraphManager instance = new GraphManager();
	
	public static GraphManager getInstance() {
		return instance;
	}
	
	/**
	 * Returns a dataset with a rectangle drawn on it at position (x,y) that is size width and height and rotated angle degrees
	 *
	 * @param transform  the {@link Transform} that defines the position and rotation of the rectangle
	 * @param width      width of the rectangle
	 * @param height     height of the rectangle
	 * @param seriesName the name of the {@link XYSeries} containing the rectangle
	 * @return an {@link XYSeriesCollection} dataset containing an {@link XYSeries} with the coordinates that make up the rectangle
	 */
	public XYSeriesCollectionWithRender graphRectangle(Transform transform, double width, double height, String seriesName, Color color) {
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, true, color, new Rectangle(2, 2));
		
		
		double halfWidth = width / 2;
		double halfHeight = height / 2;
		
		XYSeries series = new XYSeries(seriesName, false);
		
		// 0------3
		// |      |
		// 1------2
		
		Transform p0 = new Transform(-halfHeight, halfWidth).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		Transform p1 = new Transform(-halfHeight, -halfWidth).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		Transform p2 = new Transform(halfHeight, -halfWidth).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		Transform p3 = new Transform(halfHeight, halfWidth).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		
		series.add(p0.getPosition().getX(), p0.getPosition().getY());
		series.add(p1.getPosition().getX(), p1.getPosition().getY());
		series.add(p2.getPosition().getX(), p2.getPosition().getY());
		series.add(p3.getPosition().getX(), p3.getPosition().getY());
		series.add(p0.getPosition().getX(), p0.getPosition().getY());
		
		dataset.addSeries(series);
		
		return dataset;
	}
	
	
	public XYSeriesCollectionWithRender graphParametric(Parametric[] parametrics, double arrowLength, double arrowWidth, String seriesName, Color color) {
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, new Rectangle(2, 2));
		
		for (double t = 0; t < 1; t += 0.01) {
			double newT = t * parametrics.length;
			
			for (int i = 0; i < parametrics.length; i++) {
				if (newT >= i && newT <= i + 1) {
					dataset.addSeries(arrowSeries(parametrics[i].getTransform(newT - i), arrowLength, arrowWidth, seriesName + "" + newT));
				}
			}
		}
		return dataset;
	}
	
	
	public XYSeriesCollectionWithRender graphParametric(Parametric parametric, double arrowLength, double arrowWidth, String seriesName, Color color) {
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, new Rectangle(2, 2));
		for (double t = 0; t < 1; t += 0.01) {
			dataset.addSeries(arrowSeries(parametric.getTransform(t), arrowLength, arrowWidth, seriesName + "" + t));
		}
		return dataset;
	}
	
	public XYSeriesCollectionWithRender graphParametricFast(Parametric parametric, double increment, String seriesName, Color color) {
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, new Rectangle(2, 2));
		XYSeries series = new XYSeries(seriesName,false);
		for (double t = 0; t < 1; t += increment) {
			Position position = parametric.getPosition(t);
			series.add(position.getX(),position.getY());
		}
		dataset.addSeries(series);
		return dataset;
	}
	
	/**
	 * Returns an {@link XYSeriesCollection} with an arrow drawn on it starting at position (x,y), extending length long, having an arrow pointer width of arrowWidth, and pointing at angle degrees
	 *
	 * @param transform  the {@link Transform} that defines the position and rotation of the arrow
	 * @param length     length of arrow
	 * @param arrowWidth width of arrow pointer
	 * @param seriesName the name of the {@link XYSeries} containing the rectangle
	 * @return an {@link XYSeriesCollection} dataset containing an {@link XYSeries} with the coordinates that make up the arrow
	 */
	public XYSeriesCollectionWithRender graphArrow(Transform transform, double length, double arrowWidth, String seriesName, Color color) {
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, new Rectangle(2, 2));
		dataset.addSeries(arrowSeries(transform, length, arrowWidth, seriesName));
		
		return dataset;
	}
	
	public XYSeries arrowSeries(Transform transform, double length, double arrowWidth, String seriesName) {
		XYSeries series = new XYSeries(seriesName, false);
		
		double halfWidth = arrowWidth / 2;
		
		//   ^
		//  /1\
		// 2 | 3
		//   |
		//   |
		//   |
		//   0
		
		
		Transform p0 = new Transform(0, 0).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		Transform p1 = new Transform(length, 0).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		Transform p2 = new Transform(length - halfWidth, halfWidth).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		Transform p3 = new Transform(length - halfWidth, -halfWidth).transformBy(transform).rotateAround(transform.getPosition(), transform.getRotation());
		
		series.add(p0.getPosition().getX(), p0.getPosition().getY());
		series.add(p1.getPosition().getX(), p1.getPosition().getY());
		series.add(p2.getPosition().getX(), p2.getPosition().getY());
		series.add(p1.getPosition().getX(), p1.getPosition().getY());
		series.add(p3.getPosition().getX(), p3.getPosition().getY());
		return series;
	}
	
	public XYSeriesCollectionWithRender graphArc(ArcSegment arcSegment, String seriesName, Color color) {
		XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, new Rectangle(1, 1));
		
		XYSeries series = new XYSeries(seriesName, false);
		if (Math.abs(arcSegment.getRadius()) > 1000) {
			series.add(arcSegment.getStartPoint().getX(), arcSegment.getStartPoint().getY());
			series.add(arcSegment.getEndPoint().getX(), arcSegment.getEndPoint().getY());
		} else {
			double i = 180 + arcSegment.getCenter().angleTo(arcSegment.getIntermediatePoint()).getHeading();
			double initI = i;
			while (i < initI + 359) {
				Position pos = new Position(arcSegment.getCenter().getX() + Math.cos(Math.toRadians(i)) * arcSegment.getRadius(), arcSegment.getCenter().getY() + Math.sin(Math.toRadians(i)) * arcSegment.getRadius());
				if (arcSegment.isOnSegment(pos)) {
					series.add(pos.getX(), pos.getY());
				}
				
				i += 0.1;
			}
		}
		
		dataset.addSeries(series);
		return dataset;
	}
	
}
