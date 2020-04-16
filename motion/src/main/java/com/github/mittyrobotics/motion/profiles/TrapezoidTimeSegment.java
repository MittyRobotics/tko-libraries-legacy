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

package com.github.mittyrobotics.motion.profiles;

public class TrapezoidTimeSegment {
    private double tAccel;
    private double tCruise;
    private double tDecel;
    private double maxAcceleration;

    public TrapezoidTimeSegment(double tAccel, double tCruise, double tDecel, double maxAcceleration) {
        this.tAccel = tAccel;
        this.tCruise = tCruise;
        this.tDecel = tDecel;
        this.maxAcceleration = maxAcceleration;
    }

    public double getTAccel() {
        return tAccel;
    }

    public void setTAccel(double tAccel) {
        this.tAccel = tAccel;
    }

    public double getTCruise() {
        return tCruise;
    }

    public void setTCruise(double tCruise) {
        this.tCruise = tCruise;
    }

    public double getTDecel() {
        return tDecel;
    }

    public void setTDecel(double tDecel) {
        this.tDecel = tDecel;
    }

    public double getTTotal() {
        return tAccel + tCruise + tDecel;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }
}
