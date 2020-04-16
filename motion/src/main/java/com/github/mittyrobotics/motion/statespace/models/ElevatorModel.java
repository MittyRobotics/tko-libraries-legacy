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

public class ElevatorModel {
    private double acceleration;
    private double velocity;
    private double position;

    private Plant plant;

    public ElevatorModel(Motor motor, double mass, double gearReduction, double pulleyRadius, double maxVoltage) {
        this.plant = Plant.createElevatorPlant(motor,mass,pulleyRadius,gearReduction, maxVoltage, 1);
    }

    public void updateModel(double voltage, double deltaTime) {
        plant.update(new SimpleMatrix(new double[][]{{position}, {velocity}}),
                new SimpleMatrix(new double[][]{{voltage}}),deltaTime);
        SimpleMatrix states = plant.getX();
        this.acceleration = (states.get(1)-velocity)/deltaTime;
        this.position = states.get(0);
        this.velocity = states.get(1);
    }

    public Plant getPlant(){
        return plant;
    }

    public double getPosition(){
        return position;
    }

    public double getVelocity(){
        return velocity;
    }

    public double getAcceleration(){
        return acceleration;
    }

    public void setPosition(double position){
        this.position = position;
    }

    public void setVelocity(double velocity){
        this.velocity = velocity;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }
}
