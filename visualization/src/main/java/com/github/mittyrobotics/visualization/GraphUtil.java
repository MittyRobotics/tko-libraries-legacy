/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.visualization;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

import java.util.ArrayList;

public class GraphUtil {
    public static XYSeriesWithRenderer populateSeries(XYSeriesWithRenderer series, Position[] dataPoints) {
        for (int i = 0; i < dataPoints.length; i++) {
            series.add(dataPoints[i].getX(), dataPoints[i].getY());
        }
        return series;
    }

    public static Position[] rectangle(Transform centerTransform, double width, double height) {
        double halfWidth = width / 2;
        double halfHeight = height / 2;

        // 0------3
        // |      |
        // 1------2

        Transform p0 = new Transform(-halfHeight, halfWidth).transformBy(centerTransform)
                .rotateAround(centerTransform.getPosition(), centerTransform.getRotation());
        Transform p1 = new Transform(-halfHeight, -halfWidth).transformBy(centerTransform)
                .rotateAround(centerTransform.getPosition(), centerTransform.getRotation());
        Transform p2 = new Transform(halfHeight, -halfWidth).transformBy(centerTransform)
                .rotateAround(centerTransform.getPosition(), centerTransform.getRotation());
        Transform p3 = new Transform(halfHeight, halfWidth).transformBy(centerTransform)
                .rotateAround(centerTransform.getPosition(), centerTransform.getRotation());

        return new Position[]{
                p0.getPosition(), p1.getPosition(), p2.getPosition(), p3.getPosition(), p0.getPosition()
        };
    }

    public static Position[] circle(Circle circle) {
        ArrayList<Position> positions = new ArrayList<>();
        for (int i = 0; i < 360; i++) {
            Position pos = new Position(
                    circle.getCenter().getX() + Math.cos(Math.toRadians(i)) * circle.getRadius(),
                    circle.getCenter().getY() + Math.sin(Math.toRadians(i)) * circle.getRadius());
            positions.add(pos);
        }

        return positions.toArray(new Position[0]);
    }

    public static Position[] arc(ArcSegment arcSegment) {
        ArrayList<Position> positions = new ArrayList<>();

        Rotation angleToStart = new Line(arcSegment.getCenter(), arcSegment.getStartPoint()).getLineRotation();
        Rotation angleToEnd = new Line(arcSegment.getCenter(), arcSegment.getEndPoint()).getLineRotation();
        int direction = 1;
        boolean intermediateHit = false;
        int i = 0;
        while (!intermediateHit && i < 2) {
            positions.clear();
            Rotation currentAngle = angleToStart;
            Position currentPosition = new Position(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
            while (arcSegment.getIntermediatePoint().distance(currentPosition) >= 1) {
                currentAngle = currentAngle.add(Rotation.fromDegrees(.1 * direction));
                currentPosition = new Position(
                        arcSegment.getCenter().getX() + arcSegment.getRadius() * currentAngle.cos(),
                        arcSegment.getCenter().getY() + arcSegment.getRadius() * currentAngle.sin()
                );
                positions.add(currentPosition);
                if (arcSegment.getIntermediatePoint().distance(currentPosition) <= arcSegment.getRadius() / 1000) {
                    intermediateHit = true;
                }
            }
            if (!intermediateHit) {
                direction = -1;
            }
            i++;
        }

        return positions.toArray(new Position[0]);
    }
}
