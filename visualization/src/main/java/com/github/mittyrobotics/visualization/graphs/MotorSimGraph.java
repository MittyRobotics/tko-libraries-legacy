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

package com.github.mittyrobotics.visualization.graphs;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.visualization.util.XYSeriesCollectionWithRender;
import org.jfree.data.xy.XYSeries;

public class MotorSimGraph extends Graph {
    XYSeries positionSeries;
    XYSeries velocitySeries;
    XYSeries voltageSeries;
    XYSeries setpointSeries;
    XYSeries errorSeries;

    public MotorSimGraph() {
        super();
        positionSeries = new XYSeries("Position", false);
        velocitySeries = new XYSeries("Velocity", false);
        voltageSeries = new XYSeries("Voltage", false);
        setpointSeries = new XYSeries("Setpoint", false);
        errorSeries = new XYSeries("Error", false);

        final XYSeriesCollectionWithRender data = new XYSeriesCollectionWithRender();
        data.addSeries(positionSeries);
        data.addSeries(velocitySeries);
        data.addSeries(voltageSeries);
        data.addSeries(setpointSeries);
        data.addSeries(errorSeries);

        addDataset(data);

    }

    public void addPosition(double position, double time) {
        //convert to inches
        positionSeries.add(time, position * Conversions.M_TO_IN);
    }

    public void addVelocity(double velocity, double time) {
        //convert to inches
        velocitySeries.add(time, velocity * Conversions.M_TO_IN);
    }

    public void addVoltage(double voltage, double time) {
        //convert to inches
        voltageSeries.add(time, voltage);
    }

    public void addSetpoint(double setpoint, double time) {
        //convert to inches
        setpointSeries.add(time, setpoint * Conversions.M_TO_IN);
    }

    public void addError(double error, double time) {
        //convert to inches
        errorSeries.add(time, error);
    }
}
