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

import com.github.mittyrobotics.motion.controllers.StateSpaceController;
import com.github.mittyrobotics.motion.statespace.motors.Motor;
import org.ejml.simple.SimpleMatrix;

public class Plant {
    /**
     * Empty states vector. Represents number of rows and columns in the state vector, <code>x</code>, and is used to
     * reset the {@link Plant} to zero.
     */
    private final SimpleMatrix states;
    /**
     * Empty outputs vector. Represents number of rows and columns in the output vector, <code>y</code>, and is used
     * to reset the {@link Plant} to zero.
     */
    private final SimpleMatrix outputs;
    /**
     * Continuous {@link StateSpaceSystemGains}. Holds the continuous A, B, C, and D matrices of the {@link Plant}.
     */
    private final StateSpaceSystemGains continuousSystem;
    /**
     * Discrete {@link StateSpaceSystemGains}. Holds the discretized A, B, C, and D matrices of the {@link Plant}.
     */
    private final StateSpaceSystemGains discreteSystem;
    /**
     * Minimum control input matrix. Usually a 1x1 matrix containing the desired min voltage of the controller.
     */
    private final SimpleMatrix uMin;
    /**
     * Maximum control input matrix. Usually a 1x1 matrix containing the desired max voltage of the controller.
     */
    private final SimpleMatrix uMax;
    /**
     * Predicted change in time between plant update calls. This value is the initial predicted delta time, but each
     * call to the {@link StateSpaceController} is updated with the true delta time.
     */
    private double deltaTime;

    /**
     * {@link Plant} states matrix. The state of the plant is all the values that define it's current motion. This
     * could be position, velocity, acceleration, angular velocity, etc.
     */
    private SimpleMatrix x;
    /**
     * {@link Plant} outputs matrix. This is the value that can actually be measured in the plant, such as position
     * using an encoder. This is a theoretical estimate of the measurement, and can be used to either simulate the
     * plant's motion or stabilize the actual measured value with a theoretically calcualted one.
     */
    private SimpleMatrix y;

    public Plant(SimpleMatrix states, SimpleMatrix outputs, StateSpaceSystemGains continuousSystem,
                 SimpleMatrix uMin, SimpleMatrix uMax, double deltaTime) {
        this.states = states;
        this.outputs = outputs;
        this.continuousSystem = continuousSystem;
        this.discreteSystem = discretizeSystem(continuousSystem, deltaTime);
        this.uMin = uMin;
        this.uMax = uMax;
        this.deltaTime = deltaTime;
    }

    public static Plant createElevatorPlant(Motor motor, double mass, double pulleyRadius,
                                            double gearReduction, double maxVoltage, double deltaTime) {
        SimpleMatrix states, outputs, a, b, c, d, uMin, uMax;
        states = new SimpleMatrix(new double[][]{{0}, {0}}); //[[position], [velocity]]
        outputs = new SimpleMatrix(new double[][]{{0}}); //[[position]]

        double G = gearReduction;
        double Kt = motor.getKt();
        double Kv = motor.getKv();
        double R = motor.getResistance();
        double m = mass;
        double r = pulleyRadius;

        a = new SimpleMatrix(new double[][]{
                {0.0, 1.0},
                {0.0, -(G * G) * motor.getKt() / (motor.getResistance() * (r * r) * m * motor.getKv())}
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
        return new Plant(states, outputs, continuousSystem, uMin, uMax, deltaTime);
    }

    public static Plant createFlywheelPlant(Motor motor, double momentOfInertia, double gearReduction,
                                            double maxVoltage, double deltaTime) {
        SimpleMatrix states, outputs, a, b, c, d, uMin, uMax;
        states = new SimpleMatrix(new double[1][1]); //[[angular velocity]]
        states.zero();
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
        return new Plant(states, outputs, continuousSystem, uMin, uMax, deltaTime);
    }

    public static Plant createInvertedPendulumPlant(Motor motor, double cartMass, double pendulumMass,
                                                    double cartFrictionCoeff, double pendulumInertia, double gravity,
                                                    double pendulumLengthToCenterMass, double deltaTime) {
        SimpleMatrix states = null, outputs = null, a= null, b= null, c= null, d= null, uMin= null, uMax= null;

        StateSpaceSystemGains continuousSystem = new StateSpaceSystemGains(a, b, c, d);
        return new Plant(states, outputs, continuousSystem, uMin, uMax, deltaTime);
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
    }

    public void update(SimpleMatrix currentState, SimpleMatrix controlInput, double deltaTime) {
        this.x = calculateX(currentState, controlInput, deltaTime);
        this.y = calculateY(this.x, controlInput);
    }

    public SimpleMatrix calculateX(SimpleMatrix currentState, SimpleMatrix controlInput, double deltaTime) {
        StateSpaceSystemGains reDiscretizedGains = discretizeSystem(continuousSystem, deltaTime);

        SimpleMatrix reDiscretizedA = reDiscretizedGains.getA();
        SimpleMatrix reDiscretizedB = reDiscretizedGains.getB();

        return reDiscretizedA.mult(currentState)
                .plus(reDiscretizedB.mult(MatrixUtils.clamp(controlInput, uMin.get(0), uMax.get(0))));
    }

    public SimpleMatrix calculateY(SimpleMatrix currentState, SimpleMatrix controlInput) {
        return getDiscreteSystem().getC().mult(currentState)
                .plus(getDiscreteSystem().getD().mult(MatrixUtils.clamp(controlInput, uMin.get(0), uMax.get(0))));
    }

    public SimpleMatrix getStates() {
        return states;
    }

    public int getNumStates() {
        return states.numRows();
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
