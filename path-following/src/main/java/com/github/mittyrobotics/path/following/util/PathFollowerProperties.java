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

package com.github.mittyrobotics.path.following.util;

import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.generation.Path;

public class PathFollowerProperties {
    public final Path path;
    public final PathVelocityController velocityController;
    public final boolean reversed;
    public final boolean continuouslyAdaptivePath;

    public PathFollowerProperties(Path path, PathVelocityController velocityController, boolean reversed, boolean continuouslyAdaptivePath) {
        this.path = path;
        this.velocityController = velocityController;
        this.reversed = reversed;
        this.continuouslyAdaptivePath = continuouslyAdaptivePath;
    }

    public static class PurePursuitProperties {
        public final double lookaheadDistance;
        public final double curvatureSlowdownGain;
        public final double minSlowdownVelocity;

        public PurePursuitProperties(double lookaheadDistance, double curvatureSlowdownGain,
                                     double minSlowdownVelocity) {
            this.lookaheadDistance = lookaheadDistance;
            this.curvatureSlowdownGain = curvatureSlowdownGain;
            this.minSlowdownVelocity = minSlowdownVelocity;
        }
    }

    public static class RamseteProperties {
        public final double aggressiveGain;
        public final double dampingGain;

        public RamseteProperties(double aggressiveGain, double dampingGain) {
            this.aggressiveGain = aggressiveGain;
            this.dampingGain = dampingGain;
        }
    }
}
