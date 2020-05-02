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

package com.github.mittyrobotics.motion.statespace.models;

import com.github.mittyrobotics.motion.statespace.Plant;
import com.github.mittyrobotics.motion.statespace.motors.Motor;
import org.ejml.simple.SimpleMatrix;

import java.util.Random;

public class FlywheelModel {
    private double angularAcceleration;
    private double angularVelocity;
    private double position;
    private double measurementNoise;

    private Plant plant;

    public FlywheelModel(Motor motor, double momentOfInertia, double gearReduction, double maxVoltage) {
        this.plant = Plant.createFlywheelPlant(motor, momentOfInertia, gearReduction, maxVoltage, 1);
        this.measurementNoise = 0;
    }

    public void updateModel(double voltage, double deltaTime) {
        plant.update(new SimpleMatrix(new double[][]{{angularVelocity}}),
                new SimpleMatrix(new double[][]{{voltage}}), deltaTime);
        SimpleMatrix states = plant.getX();
        this.angularAcceleration = (states.get(0) - angularVelocity) / deltaTime;
        this.angularVelocity = states.get(0);
        this.position += angularVelocity * deltaTime;
    }

    private double calculateMeasurementNoise(double measurementNoise) {
        return (measurementNoise != 0 ? ((new Random().nextDouble() - 0.5) * measurementNoise) : 0);
    }

    public Plant getPlant() {
        return plant;
    }

    public double getAngularVelocity() {
        return angularVelocity + calculateMeasurementNoise(measurementNoise);
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public double getAngularAcceleration() {
        return angularAcceleration + calculateMeasurementNoise(measurementNoise);
    }

    public void setAngularAcceleration(double angularAcceleration) {
        this.angularAcceleration = angularAcceleration;
    }

    public double getPosition() {
        return position;
    }

    public double getMeasurementNoise() {
        return measurementNoise;
    }

    public void setMeasurementNoise(double measurementNoise) {
        this.measurementNoise = measurementNoise;
    }

}
