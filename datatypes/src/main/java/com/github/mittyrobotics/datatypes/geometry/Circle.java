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

package com.github.mittyrobotics.datatypes.geometry;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

/**
 * Represents a 2d circle on a standard cartesian coordinate plane.
 */
public class Circle {
    private Position center;
    private double radius;

    /**
     * Constructs a {@link Circle} given a center {@link Position} and a radius
     *
     * @param center the center {@link Position} of the circle
     * @param radius the radius of the circle
     */
    public Circle(Position center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    /**
     * Constructs a {@link Circle} given a a tangent point {@link Transform} and a radius
     *
     * @param tangentPoint a tangent point of the circle represented by a {@link Transform}
     * @param radius       the radius of the circle
     */
    public Circle(Transform tangentPoint, double radius) {
        Circle circle = getCircleFromTangentAndRadius(tangentPoint, radius);
        this.center = circle.center;
        this.radius = circle.getRadius();
    }

    /**
     * Constructs a {@link Circle} given three {@link Position}s that it intersects
     *
     * @param p1 first {@link Position}
     * @param p2 second {@link Position}
     * @param p3 third {@link Position}
     */
    public Circle(Position p1, Position p2, Position p3) {
        Circle circle = getCircleFromPoints(p1, p2, p3);
        this.center = circle.center;
        this.radius = circle.radius;
    }

    /**
     * Constructs a circle given a {@link Transform} tangent point and a {@link Position} that it intersects
     * <p>
     * The tangent point contains a {@link Position} which is the point of tangency and a {@link Rotation} that defines
     * the angle of the tangent line.
     *
     * @param tangentPoint the {@link Transform} that defines the point of tangency and tangent line
     * @param intersection another point that the circle intersects
     */
    public Circle(Transform tangentPoint, Position intersection) {
        Circle circle = getCircleFromTangentAndIntersection(tangentPoint, intersection);
        this.center = circle.center;
        this.radius = circle.radius;
    }

    /**
     * Returns the {@link Circle} defined by a tangent point {@link Transform} and the radius
     *
     * @param tangentPoint a tangent point {@link Transform}.
     * @param radius       the radius of the circle.
     * @return the {@link Circle} defined by a tangent point {@link Transform} and the radius
     */
    public Circle getCircleFromTangentAndRadius(Transform tangentPoint, double radius) {
        Transform perpendicuarTransform = new Transform(tangentPoint.getPosition(),
                tangentPoint.getRotation().rotateBy(90));
        Position center = new Position(perpendicuarTransform.getRotation().cos() * radius,
                perpendicuarTransform.getRotation().sin() * radius).add(perpendicuarTransform.getPosition());
        return new Circle(center, radius);
    }

    /**
     * Returns the {@link Circle} defined by the three {@link Position}s
     * <p>
     * Finds the center of the circle using the circumcenter formula. https://www.desmos.com/calculator/tditgey5ot
     *
     * @param p1 the first {@link Position}
     * @param p2 the second {@link Position}
     * @param p3 the third {@link Position}
     * @return a new {@link Circle} that intersects all three {@link Position}s
     */
    public Circle getCircleFromPoints(Position p1, Position p2, Position p3) {
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        double x3 = p3.getX();
        double y3 = p3.getY();

        double cX =
                (x1 * x1 * y2 - x1 * x1 * y3 - x2 * x2 * y1 + x2 * x2 * y3 + x3 * x3 * y1 - x3 * x3 * y2 + y1 * y1 * y2
                        - y1 * y2 * y2 - y1 * y1 * y3 + y1 * y3 * y3 + y2 * y2 * y3 - y2 * y3 * y3) / (2 * (x1 * y2 - x3
                        * y2 - x1 * y3 - x2 * y1 + x3 * y1 + x2 * y3));
        double cY =
                (x1 * x1 * x2 - x1 * x1 * x3 - x1 * x2 * x2 + x1 * x3 * x3 - x1 * y2 * y2 + x1 * y3 * y3 + x2 * x2 * x3
                        - x2 * x3 * x3 + x2 * y1 * y1 - x2 * y3 * y3 - x3 * y1 * y1 + x3 * y2 * y2) / (2 * (-x1 * y2 +
                        x1 * y3 + x2 * y1 - x2 * y3 - x3 * y1 + x3 * y2));

        double radius = Math.sqrt(Math.pow(cX - x1, 2) + Math.pow(cY - y1, 2));
        return new Circle(new Position(cX, cY), radius);
    }

    /**
     * Returns a circle that is tangent to the <code>tangentPoint</code> {@link Transform} and intersects the
     * <code>intersection</code> {@link Position}
     * <p>
     * The tangent point contains a {@link Position} which is the point of tangency and a {@link Rotation} that defines
     * the angle of the tangent line. https://www.desmos.com/calculator/dihvevz2n8
     *
     * @param tangentPoint the {@link Transform} that defines the point of tangency and tangent line
     * @param intersection another point that the circle intersects
     * @return a new {@link Circle} that is tangent to the <code>tangentPoint</code> {@link Transform} and intersects
     * the <code>intersection</code> {@link Position}
     */
    public Circle getCircleFromTangentAndIntersection(Transform tangentPoint, Position intersection) {
        double a = tangentPoint.getPosition().getX();
        double b = tangentPoint.getPosition().getY();
        double c = intersection.getX();
        double d = intersection.getY();
        Rotation theta = tangentPoint.getRotation();

        double cX =
                ((b + d) * (d - b) * theta.tan() - (a + c) * (a - c) * theta.tan() - 2 * (d - b) * theta.tan() * b - 2
                        * (d - b) * a) / (2 * (theta.tan() * (c - a) + b - d));
        double cY =
                (b + d) / 2 - (c - a) / (d - b) * (cX - ((a + c) * (theta.tan() * (c - a) + b - d)) / (2 * (theta.tan()
                        * (c - a) + b - d)));

        Position center = new Position(cX, cY);
        double distance = center.distance(tangentPoint.getPosition());

        if (Double.isNaN(distance)) {
            center = new Position(2e16, 2e16);
            distance = 2e16;
        }

        return new Circle(center, distance);
    }

    /**
     * Finds the two {@link Position}s of intersection between two {@link Circle}s, this {@link Circle} and
     * <code>other</code>.
     * <p>
     * This returns an array, which could either contain 0, 1, or 2 points. Make sure to check how many points of
     * intersection there are in the array after using the function.
     * <p>
     * https://www.desmos.com/calculator/ymkxfwsa1k
     *
     * @param other the other {@link Circle}
     * @return an array containing the points of intersection between the two {@link Circle}s
     */
    public Position[] circleCircleIntersection(Circle other) {
        double a = getCenter().getX();
        double b = getCenter().getY();
        double c = other.getCenter().getX();
        double d = other.getCenter().getY();
        double r = getRadius();
        double s = other.getRadius();

        double R = getCenter().distance(other.getCenter());

        //If there are no points of intersection, return an empty array.
        if (R > r + s) {
            return new Position[]{};
        }

        double a1 = Math.acos((r * r + R * R - s * s) / (2 * r * R));
        double v1 = Math.atan2(d - b, c - a);

        double v = r * Math.cos(a1 + v1);
        double g = r * Math.sin(a1 + v1);
        double k = r * Math.cos(a1 - v1);
        double j = r * Math.sin(a1 - v1);

        //this center is on the left of other center
        if (a < c) {
            double x = a + v;
            double y = b + g;
            double x1 = a + k;
            double y1 = b - j;
            return new Position[]{
                    new Position(x, y),
                    new Position(x1, y1)
            };
        }
        //this center is on the right of other center
        else {
            double x = a - v;
            double y = b - g;
            double x1 = a - k;
            double y1 = b - j;
            return new Position[]{
                    new Position(x, y),
                    new Position(x1, y1)
            };
        }
    }

    /**
     * Returns the {@link Line} of tangency at the {@link Position}.
     *
     * @param pointOfTangency the {@link Position} on the circle to find the {@link Line} of tangency.
     * @return the {@link Line} of tangency.
     */
    public Line getTangentLineAtPoint(Position pointOfTangency) {
        Line centerToPointLine = new Line(getCenter(), pointOfTangency);
        return centerToPointLine.getParallelLine(pointOfTangency);
    }

    /**
     * Finds the closest point on this {@link Circle} to the <code>referencePosition</code>.
     * <p>
     * This is done by finding the line that intersects the center of this {@link Circle} and the
     * <code>referencePosition</code>. https://www.desmos.com/calculator/mfywvuunva
     *
     * @param referencePosition the {@link Position} to find the closest point to.
     * @return the closest {@link Position} to the <code>referencePosition</code>.
     */
    public Position getClosestPoint(Position referencePosition) {
        //Find the line intersecting with the circle center and the reference position
        Line line = new Line(center, referencePosition);

        //Find all the points of intersection between the circle and the reference position
        Position[] positions = circleLineIntersection(line);

        //Find the closest point out of the intersection points to the reference position
        double currentClosest = 9999;
        Position currentClosestPosition = null;
        for (int i = 0; i < positions.length; i++) {
            if (positions[i].distance(referencePosition) < currentClosest) {
                currentClosest = positions[i].distance(referencePosition);
                currentClosestPosition = positions[i];
            }
        }

        return currentClosestPosition;
    }

    /**
     * Finds the two {@link Position}s of intersection between this {@link Circle} and {@link Line} <code>other</code>.
     * <p>
     * This returns an array, which could either contain 0, 1, or 2 points. Make sure to check how many points of
     * intersection there are in the array after using the function.
     * <p>
     * https://www.desmos.com/calculator/dwwixupcxt
     *
     * @param line the other {@link Line}
     * @return an array containing the points of intersection between the {@link Line} <code>other</code> and this
     * {@link Circle}
     */
    public Position[] circleLineIntersection(Line line) {
        double a = getCenter().getX();
        double b = getCenter().getY();
        double r = getRadius();
        double c = line.getFirstPoint().getX();
        double d = line.getFirstPoint().getY();
        double f = line.getSecondPoint().getX();
        double g = line.getSecondPoint().getY();
        double m = line.getSlope();


        double v = (a + f * m * m + b * m - g * m) / (m * m + 1);
        double v1 = (m * a + b * m * m + g - f * m) / (m * m + 1);
        double sqrt = Math.sqrt(r * r - Math.pow(a - v, 2) - Math.pow(b - v1, 2));
        double sqrt1 = Math.sqrt(Math.pow(g - d, 2) + Math.pow(f - c, 2));

        Position pos1 = new Position(
                v + (f - c) * sqrt / sqrt1,
                v1 + (g - d) * sqrt / sqrt1
        );
        Position pos2 = new Position(
                v - (f - c) * sqrt / sqrt1,
                v1 - (g - d) * sqrt / sqrt1
        );

        //If any of these values look bad, it most likely means there is no intersection and we should return an empty
        //array
        if (Double.isNaN(pos1.getX()) || Double.isNaN(pos2.getX()) || Double.isNaN(pos1.getY()) ||
                Double.isNaN(pos2.getY()) || Double.isInfinite(pos1.getX()) || Double.isInfinite(pos2.getX()) ||
                Double.isInfinite(pos1.getY()) || Double.isInfinite(pos2.getY())) {
            return new Position[]{};
        }

        return new Position[]{
                pos1,
                pos2
        };
    }

    /**
     * Determines whether or not <code>point</code> is on the circle.
     *
     * @param point the {@link Position} to determine if it is on the circle or not
     * @return whether or not <code>point</code> is on the circle.
     */
    public boolean isOnCircle(Position point) {
        return isOnCircle(point, 0.01);
    }

    /**
     * Determines whether or not <code>point</code> is on the circle given a certain <code>tolerance</code>.
     *
     * @param point     the {@link Position} to determine if it is on the circle or not
     * @param tolerance the tolerance of how far the <code>point</code> can be off the circle.
     * @return whether or not <code>point</code> is on the circle.
     */
    public boolean isOnCircle(Position point, double tolerance) {
        return Math.abs(point.distance(center) - radius) < tolerance;
    }

    public Position getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }
}
