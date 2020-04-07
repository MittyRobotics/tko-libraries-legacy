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

package com.github.mittyrobotics.motion.statespace;

import com.github.mittyrobotics.motion.statespace.motors.Motor;
import org.ejml.simple.SimpleMatrix;

public class Plant {
    private final SimpleMatrix states;
    private final SimpleMatrix inputs;
    private final SimpleMatrix outputs;
    private final StateSpaceSystemGains continuousSystem;
    private final StateSpaceSystemGains discreteSystem;
    private final SimpleMatrix uMin;
    private final SimpleMatrix uMax;
    private double deltaTime;

    private SimpleMatrix x;
    private SimpleMatrix y;
    private SimpleMatrix uDelayed;

    public Plant(SimpleMatrix states, SimpleMatrix inputs, SimpleMatrix outputs, StateSpaceSystemGains continuousSystem,
                 SimpleMatrix uMin, SimpleMatrix uMax, double deltaTime) {
        this.states = states;
        this.inputs = inputs;
        this.outputs = outputs;
        this.continuousSystem = continuousSystem;
        this.discreteSystem = discretizeSystem(continuousSystem, deltaTime);
        this.uMin = uMin;
        this.uMax = uMax;
        this.deltaTime = deltaTime;
    }

    public static Plant createElevatorPlant(Motor motor, double mass, double pulleyRadius,
                                            double gearReduction, double maxVoltage, double deltaTime) {
        SimpleMatrix states, inputs, outputs, a, b, c, d, uMin, uMax;
        states = new SimpleMatrix(new double[][]{{0}, {0}}); //[[position], [velocity]]
        inputs = new SimpleMatrix(new double[][]{{0}}); //[[voltage]]
        outputs = new SimpleMatrix(new double[][]{{0}}); //[[position]]

        double G = gearReduction;
        double Kt = motor.getKt();
        double Kv = motor.getKv();
        double R = motor.getResistance();
        double m = mass;
        double r = pulleyRadius;

        a = new SimpleMatrix(new double[][]{
                {0.0, 1.0},
                {0.0, (-(G * G) * Kt) / (R * (r * r) * m * Kv)}
        });

        b = new SimpleMatrix(new double[][]{
                {0.0},
                {(G * Kt) / (R * r * m)}
        });

        c = new SimpleMatrix(new double[][]{
                {1.0, 0.0}
        });

        d = new SimpleMatrix(new double[][]{
                {0.0}
        });

        uMin = new SimpleMatrix(new double[][]{{-maxVoltage}});
        uMax = new SimpleMatrix(new double[][]{{maxVoltage}});

        StateSpaceSystemGains continuousSystem = new StateSpaceSystemGains(a, b, c, d);
        return new Plant(states, inputs, outputs, continuousSystem, uMin, uMax, deltaTime);
    }

    public static Plant createFlywheelPlant(Motor motor, double momentOfInertia, double gearReduction,
                                            double maxVoltage, double deltaTime) {
        SimpleMatrix states, inputs, outputs, a, b, c, d, uMin, uMax;
        states = new SimpleMatrix(new double[1][1]); //[[angular velocity]]
        states.zero();
        inputs = new SimpleMatrix(new double[1][1]); //[[voltage]]
        inputs.zero();
        outputs = new SimpleMatrix(new double[1][1]); //[[angular velocity]]
        outputs.zero();

        double G = gearReduction;
        double Kt = motor.getKt();
        double Kv = motor.getKv();
        double R = motor.getResistance();
        double J = momentOfInertia;

        a = new SimpleMatrix(new double[1][1]);
        a.fill(-(G * G * Kt) / (Kv * R * J));

        b = new SimpleMatrix(new double[1][1]);
        b.fill((G * Kt) / (R * J));

        c = SimpleMatrix.identity(1);

        d = new SimpleMatrix(new double[1][1]);
        d.zero();

        uMin = new SimpleMatrix(new double[][]{{-maxVoltage}});
        uMax = new SimpleMatrix(new double[][]{{maxVoltage}});

        StateSpaceSystemGains continuousSystem = new StateSpaceSystemGains(a, b, c, d);
        return new Plant(states, inputs, outputs, continuousSystem, uMin, uMax, deltaTime);
    }

    public StateSpaceSystemGains discretizeSystem(StateSpaceSystemGains input, double dt) {

        SimpleMatrix emUpper = MatrixUtils.hStack(input.getA(), input.getB());
        SimpleMatrix lowerA = new SimpleMatrix(new double[input.getB().numCols()][input.getA().numRows()]);
        lowerA.zero();
        SimpleMatrix lowerB = new SimpleMatrix(new double[input.getB().numCols()][input.getB().numCols()]);
        lowerB.zero();
        SimpleMatrix emLower = MatrixUtils.hStack(lowerA, lowerB);

        SimpleMatrix em = MatrixUtils.vStack(emUpper, emLower);

        SimpleMatrix ms = MatrixUtils.expm(MatrixUtils.multByDouble(em, dt));

        ms = MatrixUtils.cut(0, input.getA().numRows(), 0, ms.numCols(), ms);

        SimpleMatrix ad = MatrixUtils.cut(0, ms.numRows(), 0, input.getA().numCols(), ms);

        SimpleMatrix bd = MatrixUtils.cut(0, ms.numRows(), input.getA().numCols(), ms.numCols(), ms);

        return new StateSpaceSystemGains(ad, bd, input.getC(), input.getD());
    }

    public void reset() {
        this.x = new SimpleMatrix(new double[states.numRows()][states.numCols()]);
        this.y = new SimpleMatrix(new double[outputs.numRows()][outputs.numCols()]);
        this.uDelayed = new SimpleMatrix(new double[inputs.numRows()][inputs.numCols()]);
    }

    public void update(SimpleMatrix currentState, SimpleMatrix controlInput, double deltaTime) {
        this.x = calculateX(currentState, uDelayed, deltaTime);
        this.y = calculateY(this.x, uDelayed);
        this.uDelayed = controlInput;
    }

    public SimpleMatrix calculateX(SimpleMatrix currentState, SimpleMatrix controlInput, double deltaTime) {
        StateSpaceSystemGains reDiscretizedGains = discretizeSystem(continuousSystem, deltaTime);

        SimpleMatrix reDiscretizedA = reDiscretizedGains.getA();
        SimpleMatrix reDiscretizedB = reDiscretizedGains.getB();

        return (reDiscretizedA.mult(currentState))
                .plus(reDiscretizedB.mult(MatrixUtils.clamp(controlInput, uMin.get(0), uMax.get(0))));
    }

    public SimpleMatrix calculateY(SimpleMatrix currentState, SimpleMatrix controlInput) {
        return getContinuousSystem().getC().mult(currentState)
                .plus(getContinuousSystem().getD().mult(MatrixUtils.clamp(controlInput, uMin.get(0), uMax.get(0))));
    }

    public SimpleMatrix getStates() {
        return states;
    }

    public SimpleMatrix getInputs() {
        return inputs;
    }

    public SimpleMatrix getOutputs() {
        return outputs;
    }

    public StateSpaceSystemGains getContinuousSystem() {
        return continuousSystem;
    }

    public StateSpaceSystemGains getDiscreteSystem() {
        return discreteSystem;
    }

    public SimpleMatrix getuMin() {
        return uMin;
    }

    public SimpleMatrix getuMax() {
        return uMax;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public SimpleMatrix getX() {
        return x;
    }

    public void setX(SimpleMatrix x) {
        this.x = x;
    }

    public SimpleMatrix getY() {
        return y;
    }

    public void setY(SimpleMatrix y) {
        this.y = y;
    }
}
