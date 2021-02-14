/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.motion.controllers;


import com.github.mittyrobotics.path.generation.Path;

import java.util.ArrayList;

public class PathVelocityController {
    private final double maxAcceleration;
    private final double maxDeceleration;
    private final double maxVelocity;
    private final double startVelocity;
    private final double endVelocity;
    private final SafeVelocityController safeVelocityController;
    private double curvatureSlowdownGain;
    private double minSlowdownVelocity;
    private ArrayList<VelocityAndDistance> velocityAndDistances;

    public PathVelocityController(double maxAcceleration, double maxDeceleration, double maxVelocity,
                                  double startVelocity, double endVelocity, double curvatureSlowdownGain, double minSlowdownVelocity) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.curvatureSlowdownGain = curvatureSlowdownGain;
        this.minSlowdownVelocity = minSlowdownVelocity;
        this.safeVelocityController = new SafeVelocityController(maxAcceleration, maxDeceleration, maxVelocity);
        velocityAndDistances = new ArrayList<>();
    }

    private static double calculateSlowdownVelocity(double curvature, double curvatureSlowdownGain,
                                                    double currentVelocity,
                                                    double minSlowdownVelocity) {
        if (curvatureSlowdownGain <= 2e-9) {
            return currentVelocity;
        }
        double vel = Math.min(Math.max(minSlowdownVelocity, Math.abs(curvatureSlowdownGain / curvature)), 5);
        if (vel >= 2e9) {
            return currentVelocity;
        }
        return vel;
    }

    public static double calculateDistanceToSlowdown(double currentVelocity, double slowdownVelocity, double maxDeceleration) {
        double v = currentVelocity - slowdownVelocity;
        return v * v / (2 * maxDeceleration);
    }

    public static double calculateMaxVelocityFromDistance(double endVelocity, double distance, double maxDeceleration) {
        return Math.sqrt(endVelocity * endVelocity + 2 * maxDeceleration * distance);
    }

    public double getVelocity(Path path, double previousVelocity, double traveledDistance, double deltaTime) {
        //Calculate max velocity to end
        double distanceToEnd = Math.max(0, path.getGaussianQuadratureLength() - traveledDistance);
        double maxVelocityToEnd = calculateMaxVelocityFromDistance(0.0, distanceToEnd, maxDeceleration);
        //Calculate initial trapezoidal velocity from safe velocity controller
        double velocity = safeVelocityController.getVelocity(previousVelocity, Math.min(maxVelocity, maxVelocityToEnd), deltaTime);

        //Calculate preview distance to slowdown from current velocity to zero velocity
        double previewDistance = calculateDistanceToSlowdown(previousVelocity, 0.0, maxDeceleration);
        //Get curvature at current point and preview point
        double curvature = path.getCurvature(path.getParameterFromLength(traveledDistance));
        double curvatureAtPreview = path.getCurvature(path.getParameterFromLength(traveledDistance + previewDistance));
        //Calculate slowdown velocity at current point and preview point
        double slowdownVelocity = calculateSlowdownVelocity(curvature, curvatureSlowdownGain, previousVelocity, minSlowdownVelocity);
        double slowdownVelocityAtPreview = calculateSlowdownVelocity(curvatureAtPreview, curvatureSlowdownGain, previousVelocity, minSlowdownVelocity);

        //Remove old array values that we have traveled past
        removeOldArrayValues(traveledDistance);
        //Add new preview velocity to array
        velocityAndDistances.add(new VelocityAndDistance(slowdownVelocityAtPreview, traveledDistance + previewDistance));

        //Get minimum velocity from the array required to slowdown to a future velocity
        double minVelocityToSlowdown = getMinVelFromArray(traveledDistance);

        //If min velocity to slowdown is less than the previous velocity, we want to slowdown
        if (minVelocityToSlowdown < previousVelocity) {
            velocity = Math.min(velocity, minVelocityToSlowdown);
        }
        //If new velocity minus slowdown velocity is less than max deceleration, we want to use slowdown velocity instead. This avoids stepping glitches with the minVelocityToSlowdown.
        if (Math.abs(velocity - slowdownVelocity) < maxDeceleration) {
            velocity = Math.min(velocity, slowdownVelocity);
        }

        return velocity;
    }

    private double getMinVelFromArray(double currentDistance) {
        double maxVel = 9999;
        for (VelocityAndDistance velocityAndDistance : velocityAndDistances) {
            double vel = calculateMaxVelocityFromDistance(velocityAndDistance.velocity, velocityAndDistance.distance - currentDistance, maxDeceleration);
            maxVel = Math.min(vel, maxVel);
        }
        return maxVel;
    }

    private void removeOldArrayValues(double currentDistance) {
        velocityAndDistances.removeIf(velocityAndDistance -> velocityAndDistance.distance < currentDistance);
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getStartVelocity() {
        return startVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public SafeVelocityController getSafeVelocityController() {
        return safeVelocityController;
    }

    public static class VelocityAndDistance {
        public double velocity;
        public double distance;

        public VelocityAndDistance(double velocity, double distance) {
            this.velocity = velocity;
            this.distance = distance;
        }
    }
}
