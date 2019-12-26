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

package com.github.mittyrobotics.motionprofile;

import com.github.mittyrobotics.datatypes.motion.VelocityConstraints;

public class PathVelocityController {
    private final VelocityConstraints velocityConstraints;
    private final double startVelocity;
    private final double endVelocity;
    SafeVelocityController safeVelocityController;


    public PathVelocityController(VelocityConstraints velocityConstraints, double startVelocity, double endVelocity) {
        this.velocityConstraints = velocityConstraints;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        safeVelocityController = new SafeVelocityController(velocityConstraints);
    }

    public double getVelocity(double currentVelocity, double distanceToEnd, double deltaTime) {
        double maxDistanceVelocity = Math.sqrt(2 * velocityConstraints.getMaxDeceleration() * distanceToEnd);
        double desiredVelocity = Math.min(velocityConstraints.getMaxVelocity(), maxDistanceVelocity);

        return safeVelocityController.getVelocity(currentVelocity, desiredVelocity, deltaTime);
    }

    public VelocityConstraints getVelocityConstraints() {
        return velocityConstraints;
    }

    public double getStartVelocity() {
        return startVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }
}
