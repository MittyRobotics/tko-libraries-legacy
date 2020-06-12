/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.motion.controllers;

import org.ejml.simple.SimpleMatrix;

import javax.sound.sampled.Control;

public class ControlLoopInput {
    private SimpleMatrix values;

    public ControlLoopInput(){

    }

    public ControlLoopInput(double... values){
        this.values = new SimpleMatrix(values.length, 1, true, values);
    }

    public SimpleMatrix getValues() {
        return values;
    }

    public double getValue(int index) {
        return getValues().get(index);
    }

    public void setValues(SimpleMatrix values) {
        this.values = values;
    }

    public static class VoltageOutput extends ControlLoopInput{
        public VoltageOutput(double voltage){
            super(voltage);
        }

        public double getVoltage(){
            return getValue(0);
        }
    }
}
