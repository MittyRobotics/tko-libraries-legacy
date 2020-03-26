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

import org.jfree.data.xy.XYDataItem;
import org.jfree.data.xy.XYSeries;

public class MotorGraph extends Graph {
    private String positionKey = "Position";
    private String velocityKey = "Velocity";
    private String accelerationKey = "Acceleration";
    private String voltageKey = "Voltage";
    private String setpointKey = "Setpoint";
    private String errorKey = "Error";

    public MotorGraph(String name, String yName, String xName) {
        super(name, yName, xName);
        XYSeriesWithRenderer positionSeries = new XYSeriesWithRenderer(positionKey);
        positionSeries.setShowLines(false);
        XYSeriesWithRenderer velocitySeries = new XYSeriesWithRenderer(velocityKey);
        velocitySeries.setShowLines(false);
        XYSeries accelerationSeries = new XYSeries(accelerationKey, false);
        XYSeries voltageSeries = new XYSeries(voltageKey, false);
        XYSeries setpointSeries = new XYSeries(setpointKey, false);
        XYSeries errorSeries = new XYSeries(errorKey, false);

        addSeries(positionSeries);
        addSeries(velocitySeries);
        addSeries(accelerationSeries);
        addSeries(voltageSeries);
        addSeries(setpointSeries);
        addSeries(errorSeries);
    }

    public MotorGraph() {
        this("Graph", "y", "x");
    }

    public void addPosition(double position, double time) {
        //convert to inches
        addToSeries(positionKey, new XYDataItem(time, position));
    }

    public void addVelocity(double velocity, double time) {
        //convert to inches
        addToSeries(velocityKey, new XYDataItem(time, velocity));
    }

    public void addAcceleration(double acceleration, double time) {
        //convert to inches
        addToSeries(accelerationKey, new XYDataItem(time, acceleration));
    }

    public void addVoltage(double voltage, double time) {
        //convert to inches
        addToSeries(voltageKey, new XYDataItem(time, voltage));
    }

    public void addSetpoint(double setpoint, double time) {
        //convert to inches
        addToSeries(setpointKey, new XYDataItem(time, setpoint));
    }

    public void addError(double error, double time) {
        //convert to inches
        addToSeries(errorKey, new XYDataItem(time, error));
    }
}
