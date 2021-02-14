/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion.controllers;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.geometry.Line;
import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Transform;
import com.github.mittyrobotics.motion.pathfollowing.PathFollower;
import com.github.mittyrobotics.motion.pathfollowing.PathFollowerProperties;
import com.github.mittyrobotics.visualization.Graph;
import org.jfree.data.xy.XYDataItem;

import java.util.ArrayList;

public class PurePursuitController extends PathFollower {
    public static double DEFAULT_LOOKAHEAD_DISTANCE = .5;
    public static double DEFAULT_CURVATURE_SLOWDOWN_GAIN = 0.0;
    public static double DEFAULT_MIN_SLOWDOWN_VELOCITY = 0.0;

    private PathFollowerProperties.PurePursuitProperties purePursuitProperties;

    private Circle pursuitCircle;
    private Position lookaheadPoint;

    private Graph graph;
    private double t;

    private ArrayList<VelocityAndDistance> velocityAndDistances;


    public double getCurvatureSlowdownVelocity() {
        return curvatureSlowdownVelocity;
    }

    private double curvatureSlowdownVelocity;

    public PurePursuitController(PathFollowerProperties properties,
                                 PathFollowerProperties.PurePursuitProperties purePursuitProperties) {
        super(properties);
        this.purePursuitProperties = purePursuitProperties;
//        graph = new Graph();
        this.velocityAndDistances = new ArrayList<>();
    }

    /**
     * Calculates the velocity of the robot after applying the <code>curvatureSlowdownGain</code> to the
     * <code>curvature</code>.
     *
     * @param curvature           the curvature (1/radius) of the arc the robot is following.
     * @param currentVelocity     the current velocity that the robot is or should be moving at.
     * @param minSlowdownVelocity the minimum allowed velocity to slow down to.
     * @return the velocity of the robot after applying the <code>curvatureSlowdownGain</code> to the
     * <code>curvature</code>.
     */
    private static double calculateSlowdownVelocity(double curvature, double curvatureSlowdownGain,
                                                    double currentVelocity,
                                                    double minSlowdownVelocity) {
        if (curvatureSlowdownGain <= 2e-9) {
            return currentVelocity;
        }
        double vel = Math.min(Math.max(minSlowdownVelocity, Math.abs(curvatureSlowdownGain / curvature)), 5);
        if(vel >= 2e9){
            return currentVelocity;
        }
        return vel;
    }

    @Override
    public DrivetrainState calculate(Transform robotTransform, DrivetrainState currentDrivetrainVelocities,
                                     double deltaTime) {
        double currentVelocity = getPreviousCalculatedVelocity();
        double currentDistance = getTraveledDistance();
        double lookaheadDistance = purePursuitProperties.lookaheadDistance;

        double minSlowdownVelocity = 0.2;
        double curvatureSlowdownGain = .4;

        Position targetPosition = getCurrentPath().getTransformFromLength(getTraveledDistance() + lookaheadDistance).getPosition();
//
//        double distanceToSlowdown = getProperties().velocityController.calculateDistanceToSlowdown(getPreviousCalculatedVelocity(), 0, getProperties().velocityController.getMaxDeceleration());
//
//        double curvature = getCurrentPath().getCurvature(getCurrentPath().getParameterFromLength(getTraveledDistance()));
//        double curvatureAtLookahead = getCurrentPath().getCurvature(getCurrentPath().getParameterFromLength(getTraveledDistance()+distanceToSlowdown));
//        double slowdownVelocityAtLookahead = calculateSlowdownVelocity(curvatureAtLookahead, curvatureSlowdownGain, getPreviousCalculatedVelocity(), minSlowdownVelocity);
//        double slowdownVelocityAtCurvature = calculateSlowdownVelocity(curvature, curvatureSlowdownGain, getPreviousCalculatedVelocity(), minSlowdownVelocity);
//        curvatureSlowdownVelocity = slowdownVelocityAtCurvature;
//
//        removeOldValues(velocityAndDistances, getTraveledDistance());
//
//        velocityAndDistances.add(new VelocityAndDistance(slowdownVelocityAtLookahead,getTraveledDistance() + distanceToSlowdown));
//
//        double maxVelFromArray = getMaxVelFromArray(velocityAndDistances, getTraveledDistance());
//
////        if((curvatureSlowdownVelocity-getPreviousCalculatedVelocity())/deltaTime < getProperties().velocityController.getMaxDeceleration()){
////            maxVelFromArray = curvatureSlowdownVelocity;
////        }
//
//        t = getTraveledDistance();
//        //Calculate the robot velocity using the path velocity controller
//        double robotVelocity = getProperties().velocityController
//                .getVelocity(getCurrentPath(), getPreviousCalculatedVelocity(),  getTraveledDistance(),
//                        deltaTime);
//
//        if(maxVelFromArray < getPreviousCalculatedVelocity()){
//            robotVelocity = Math.min(robotVelocity, maxVelFromArray);
//        }
//
//
//        if(Math.abs(robotVelocity-curvatureSlowdownVelocity) < getProperties().velocityController.getMaxDeceleration()){
//            robotVelocity = Math.min(curvatureSlowdownVelocity, robotVelocity);
//        }

        double robotVelocity = getProperties().velocityController.getVelocity(getCurrentPath(), getPreviousCalculatedVelocity(), getTraveledDistance(), deltaTime);

//        graph.addToSeries("theoretical vel", new XYDataItem(t, robotVelocity));
//        graph.addToSeries("slowdown vel", new XYDataItem(t, slowdownVelocityAtLookahead));
//        graph.addToSeries("distance to slowdown", new XYDataItem(t, distanceToSlowdown));
////        graph.addToSeries("dynamic vel", new XYDataItem(t, dynamicMaxVelocity));
//        graph.addToSeries("distance to end", new XYDataItem(t, getDistanceToEnd()));
//        graph.addToSeries("curvature", new XYDataItem(t, curvatureAtLookahead));
//        graph.addToSeries("curvature slowdown", new XYDataItem(t, slowdownVelocityAtCurvature));
//        graph.addToSeries("vel + dist", new XYDataItem(velocityAndDistances.get(velocityAndDistances.size()-1).distance, velocityAndDistances.get(velocityAndDistances.size()-1).velocity));
//        graph.addToSeries("max vel from array", new XYDataItem(t, maxVelFromArray));


//        robotVelocity = Math.min(robotVelocity, slowdownVelocityAtCurvature);

        //Calculate the pursuit circle to follow, calculated by finding the circle tangent to the robot transform that
        //intersects the target position.
        this.pursuitCircle = new Circle(robotTransform, targetPosition);

        //Determine which side the robot transform is on the circle
        double side = new Line(robotTransform.getPosition(),
                robotTransform.getPosition().add(
                        new Position(
                                robotTransform.getRotation().cos() * 5,
                                robotTransform.getRotation().sin() * 5)
                )).findSide(pursuitCircle.getCenter());

        double radius = pursuitCircle.getRadius() * side;

//        double slowdownVelocity =
//                calculateSlowdownVelocity(1 / (pursuitCircle.getRadius()), purePursuitProperties.curvatureSlowdownGain,
//                        robotVelocity,
//                        purePursuitProperties.minSlowdownVelocity);

//        robotVelocity = getProperties().velocityController.getSafeVelocityController().getVelocity(getPreviousCalculatedVelocity(), slowdownVelocity, deltaTime);



        //Use differential drive kinematics to calculate the left and right wheel velocity given the base robot
        //velocity and the radius of the pursuit circle
        DrivetrainState state =
                DrivetrainState.fromLinearAndRadius(robotVelocity, radius, getProperties().trackWidth);

        if (getProperties().reversed) {
            state = state.reverse();
        }
        t += deltaTime;
        return state;
    }

    public double getMaxVelFromArray(ArrayList<VelocityAndDistance> velocityAndDistances, double currentDistance){
        double maxVel = 9999;
        for(VelocityAndDistance velocityAndDistance : velocityAndDistances){
            double vel = getProperties().velocityController.calculateMaxVelocityFromDistance(velocityAndDistance.velocity, velocityAndDistance.distance-currentDistance, getProperties().velocityController.getMaxDeceleration());
            maxVel = Math.min(vel, maxVel);
        }
        return maxVel;
    }

    public void removeOldValues(ArrayList<VelocityAndDistance> velocityAndDistances, double currentDistance){
        velocityAndDistances.removeIf(velocityAndDistance -> velocityAndDistance.distance < currentDistance);
    }

    public Circle getPursuitCircle() {
        return pursuitCircle;
    }

    public Position getLookaheadPoint(){
        return lookaheadPoint;
    }

    public static class VelocityAndDistance{
        public double velocity;
        public double distance;
        public VelocityAndDistance(double velocity, double distance){
            this.velocity = velocity;
            this.distance = distance;
        }
    }

}
