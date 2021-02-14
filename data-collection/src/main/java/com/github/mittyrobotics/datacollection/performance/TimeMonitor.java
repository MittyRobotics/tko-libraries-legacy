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

package com.github.mittyrobotics.datacollection.performance;

public class TimeMonitor {
    private long absoluteStartTime;
    private long absoluteEndTime;
    private long timeNanos;
    private String name;

    public TimeMonitor() {
        this.name = "task";
    }

    public TimeMonitor(String name) {
        this.name = name;
    }

    public void start() {
        this.absoluteStartTime = System.nanoTime();
    }

    public void end() {
        this.absoluteEndTime = System.nanoTime();
        this.timeNanos = absoluteEndTime - absoluteStartTime;
    }

    public double getNanos() {
        if (timeNanos == 0) {
            System.out.println("Time has not been measured because it has either not started or stopped");
        }
        return timeNanos;
    }

    public double getMillis() {
        if (timeNanos == 0) {
            System.out.println("Time has not been measured because it has either not started or stopped");
        }
        return timeNanos / 1E6;
    }

    public double getSeconds() {
        if (timeNanos == 0) {
            System.out.println("Time has not been measured because it has either not started or stopped");
        }
        return timeNanos / 1E9;
    }

    public void printNanos() {
        if (timeNanos == 0) {
            System.out.println("Time has not been measured because it has either not started or stopped");
        }
        System.out.println("Ellapsed time in nanoseconds to perform task " + name + ": " + timeNanos + " nanoseconds.");
    }

    public void printMillis() {
        if (timeNanos == 0) {
            System.out.println("Time has not been measured because it has either not started or stopped");
        }
        System.out.println("Ellapsed time in milliseconds to perform task " + name + ": " + getMillis() +
                " milliseconds.");
    }

    public void printSeconds() {
        if (timeNanos == 0) {
            System.out.println("Time has not been measured because it has either not started or stopped");
        }
        System.out.println("Ellapsed time in seconds to perform task " + name + ": " + getSeconds() +
                " seconds.");
    }

}
