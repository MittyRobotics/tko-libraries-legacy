package com.github.mittyrobotics.visualization;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
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
     * @param x          x position of the center of the rectangle
     * @param y          y position of the center of the rectangle
     * @param width      width of the rectangle
     * @param height     height of the rectangle
     * @param angle      the angle of the rectangle in degrees
     * @param seriesName the name of the {@link XYSeries} containing the rectangle
     * @return an {@link XYSeriesCollection} dataset containing an {@link XYSeries} with the coordinates that make up the rectangle
     */
    public XYSeriesCollectionWithRender graphRectangle(double x, double y, double width, double height, double angle, String seriesName) {
        XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, true, Color.white, new Rectangle(2, 2));

        double halfWidth = width / 2;
        double halfHeight = height / 2;

        XYSeries series = new XYSeries(seriesName, false);

        // 0------3
        // |      |
        // 1------2

        Transform origin = new Transform(x, y, angle);

        Transform p0 = new Transform(-halfHeight, halfWidth).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());
        Transform p1 = new Transform(-halfHeight, -halfWidth).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());
        Transform p2 = new Transform(halfHeight, -halfWidth).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());
        Transform p3 = new Transform(halfHeight, halfWidth).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());

        series.add(p0.getPosition().getX(), p0.getPosition().getY());
        series.add(p1.getPosition().getX(), p1.getPosition().getY());
        series.add(p2.getPosition().getX(), p2.getPosition().getY());
        series.add(p3.getPosition().getX(), p3.getPosition().getY());
        series.add(p0.getPosition().getX(), p0.getPosition().getY());

        dataset.addSeries(series);

        return dataset;
    }

    /**
     * Returns an {@link XYSeriesCollection} with an arrow drawn on it starting at position (x,y), extending length long, having an arrow pointer width of arrowWidth, and pointing at angle degrees
     *
     * @param x          x position of arrow
     * @param y          y position of arrow
     * @param length     length of arrow
     * @param arrowWidth width of arrow pointer
     * @param angle      angle of arrow
     * @param seriesName the name of the {@link XYSeries} containing the rectangle
     * @return an {@link XYSeriesCollection} dataset containing an {@link XYSeries} with the coordinates that make up the arrow
     */
    public XYSeriesCollectionWithRender graphArrow(double x, double y, double length, double arrowWidth, double angle, String seriesName, Color color) {
        XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, new Rectangle(2, 2));

        XYSeries series = new XYSeries(seriesName, false);

        Transform origin = new Transform(x, y, angle);

        double halfWidth = arrowWidth / 2;

        //   ^
        //  /1\
        // 2 | 3
        //   |
        //   |
        //   |
        //   0


        Transform p0 = new Transform(0, 0).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());
        Transform p1 = new Transform(length, 0).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());
        Transform p2 = new Transform(length - halfWidth, halfWidth).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());
        Transform p3 = new Transform(length - halfWidth, -halfWidth).transformBy(origin).rotateAround(origin.getPosition(), origin.getRotation());

        series.add(p0.getPosition().getX(), p0.getPosition().getY());
        series.add(p1.getPosition().getX(), p1.getPosition().getY());
        series.add(p2.getPosition().getX(), p2.getPosition().getY());
        series.add(p1.getPosition().getX(), p1.getPosition().getY());
        series.add(p3.getPosition().getX(), p3.getPosition().getY());

        dataset.addSeries(series);

        return dataset;
    }

    public XYSeriesCollectionWithRender graphArc(ArcSegment arcSegment, String seriesName, Color color) {
        XYSeriesCollectionWithRender dataset = new XYSeriesCollectionWithRender(true, false, color, null);

        XYSeries series = new XYSeries(seriesName, false);
        if (Math.abs(arcSegment.getRadius()) > 100) {
        	series.add(arcSegment.getStartPoint().getX(),arcSegment.getStartPoint().getY());
        	series.add(arcSegment.getEndPoint().getX(),arcSegment.getEndPoint().getY());
        } else {
            for (double i = 0; i < 360; i += 0.1) {
            	if(arcSegment.isOnSegment(new Position(arcSegment.getCenter().getX() + Math.cos(Math.toRadians(i)) * arcSegment.getRadius(), arcSegment.getCenter().getY() + Math.sin(Math.toRadians(i)) * arcSegment.getRadius()))){
					series.add(arcSegment.getCenter().getX() + Math.cos(Math.toRadians(i)) * arcSegment.getRadius(), arcSegment.getCenter().getY() + Math.sin(Math.toRadians(i)) * arcSegment.getRadius());
				}
            }
        }

        dataset.addSeries(series);
        return dataset;
    }

}
