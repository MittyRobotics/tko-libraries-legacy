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


public class PathVelocityController {
    private final double maxAcceleration;
    private final double maxDeceleration;
    private final double maxVelocity;
    private final double startVelocity;
    private final double endVelocity;
    private final SafeVelocityController safeVelocityController;

    public PathVelocityController(double maxAcceleration, double maxDeceleration, double maxVelocity,
                                  double startVelocity, double endVelocity) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.safeVelocityController = new SafeVelocityController(maxAcceleration, maxDeceleration, maxVelocity);
    }

    public double getVelocity(double currentVelocity, double distanceToEnd, double deltaTime) {
        double maxDistanceVelocity = Math.sqrt(2 * maxDeceleration * distanceToEnd);
        double desiredVelocity = Math.min(maxVelocity, maxDistanceVelocity);

        double deltaVelocity = currentVelocity - desiredVelocity;
        return safeVelocityController.getVelocity(currentVelocity, desiredVelocity, deltaTime);
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

}
