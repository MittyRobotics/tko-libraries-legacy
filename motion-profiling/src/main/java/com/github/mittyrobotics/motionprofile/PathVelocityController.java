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
    private final boolean extremeTakeoff;
    private final double extremeTakeoffMultiplier;
    private SafeVelocityController safeVelocityController;
    private boolean inTakeoffMode = true;
    private int previousSign = 0;

    public PathVelocityController(VelocityConstraints velocityConstraints, double startVelocity, double endVelocity) {
        this(velocityConstraints, startVelocity, endVelocity, false, 0);
    }

    public PathVelocityController(VelocityConstraints velocityConstraints, double startVelocity, double endVelocity,
                                  boolean extremeTakeoff) {
        this(velocityConstraints, startVelocity, endVelocity, extremeTakeoff, 2);
    }

    public PathVelocityController(VelocityConstraints velocityConstraints, double startVelocity, double endVelocity,
                                  boolean extremeTakeoff, double extremeTakeoffMultiplier) {
        this.velocityConstraints = velocityConstraints;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.safeVelocityController = new SafeVelocityController(velocityConstraints);
        this.extremeTakeoff = extremeTakeoff;
        this.extremeTakeoffMultiplier = extremeTakeoffMultiplier;
    }

    public double getVelocity(double currentVelocity, double distanceToEnd, double deltaTime) {
        double maxDistanceVelocity = Math.sqrt(2 * velocityConstraints.getMaxDeceleration() * distanceToEnd);
        double desiredVelocity = Math.min(velocityConstraints.getMaxVelocity(), maxDistanceVelocity);

        double deltaVelocity = currentVelocity - desiredVelocity;

        //Calculate if still in takeoff mode
        if (inTakeoffMode) {
            int sign = (int) Math.signum(deltaVelocity);
            inTakeoffMode = sign == previousSign || previousSign == 0;
            previousSign = sign;
        }

        //If in takeoff mode and extreme takeoff is true, multiple acceleration and deceleration by extremeTakeoffMultiplier
        if (inTakeoffMode && extremeTakeoff) {
            return safeVelocityController.getVelocity(currentVelocity, desiredVelocity, deltaTime,
                    new VelocityConstraints(velocityConstraints.getMaxAcceleration() * extremeTakeoffMultiplier,
                            velocityConstraints.getMaxDeceleration() * extremeTakeoffMultiplier,
                            velocityConstraints.getMaxVelocity()));
        }
        //Otherwise use the default velocity constraints
        else {
            return safeVelocityController.getVelocity(currentVelocity, desiredVelocity, deltaTime);
        }
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

    public boolean isExtremeTakeoff() {
        return extremeTakeoff;
    }

    public double getExtremeTakeoffMultiplier() {
        return extremeTakeoffMultiplier;
    }
}
