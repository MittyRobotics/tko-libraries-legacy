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

package com.github.mittyrobotics.motion.controllers;

import com.github.mittyrobotics.motion.statespace.KalmanFilter;
import com.github.mittyrobotics.motion.statespace.LinearQuadraticRegulator;
import com.github.mittyrobotics.motion.statespace.MatrixUtils;
import com.github.mittyrobotics.motion.statespace.Plant;
import com.github.mittyrobotics.motion.statespace.models.PulleyModel;
import com.github.mittyrobotics.motion.statespace.models.FlywheelModel;
import org.ejml.simple.SimpleMatrix;

/**
 * State space controller that controls the {@link Plant}, {@link LinearQuadraticRegulator} controller, and
 * {@link KalmanFilter} observer.
 */
public class StateSpaceController {
    private final Plant plant;
    private final LinearQuadraticRegulator controller;
    private final KalmanFilter observer;

    private SimpleMatrix nextR;

    public static StateSpaceController makeElevatorController(PulleyModel model, double modelPositionAccuracy,
                                                              double modelVelocityAccuracy, double measurementAccuracy,
                                                              double positionTolerance, double velocityTolerance,
                                                              double voltageTolerance, double qWeight) {
        Plant plant = model.getPlant();

        LinearQuadraticRegulator controller = new LinearQuadraticRegulator(plant,
                new SimpleMatrix(new double[][]{{positionTolerance, velocityTolerance}}),
                new SimpleMatrix(new double[][]{{voltageTolerance}}), qWeight);

        KalmanFilter observer = new KalmanFilter(plant,
                new SimpleMatrix(new double[][]{{modelPositionAccuracy, modelVelocityAccuracy}}),
                new SimpleMatrix(new double[][]{{measurementAccuracy}}));

        return new StateSpaceController(plant, controller, observer);
    }

    public static StateSpaceController makeFlywheelController(FlywheelModel model, double modelAngularVelocityAccuracy,
                                                              double measurementAccuracy,
                                                              double angularVelocityTolerance, double voltageTolerance,
                                                              double qWeight) {
        Plant plant = model.getPlant();

        LinearQuadraticRegulator controller = new LinearQuadraticRegulator(plant,
                new SimpleMatrix(new double[][]{{angularVelocityTolerance}}),
                new SimpleMatrix(new double[][]{{voltageTolerance}}), qWeight);

        KalmanFilter observer = new KalmanFilter(plant,
                new SimpleMatrix(new double[][]{{modelAngularVelocityAccuracy}}),
                new SimpleMatrix(new double[][]{{measurementAccuracy}}));

        return new StateSpaceController(plant, controller, observer);
    }

    public StateSpaceController(Plant plant, LinearQuadraticRegulator controller, KalmanFilter observer) {
        this.plant = plant;
        this.controller = controller;
        this.observer = observer;

        this.nextR = new SimpleMatrix(plant.getNumStates(), 0);
        reset();
    }

    public void setNextR(SimpleMatrix nextR) {
        this.nextR = nextR;
    }

    public void reset() {
        plant.reset();
        controller.reset();

        this.nextR = new SimpleMatrix(plant.getNumStates(), 0);
    }

    public SimpleMatrix calculate(SimpleMatrix measurement, SimpleMatrix reference, double deltaTime) {
        setNextR(reference);
        correct(measurement);
        predict(deltaTime);

        return getU();
    }

    public void predict(double deltaTime) {
        controller.update(getObserver().getXhat(), nextR);
        observer.predict(getController().getU(), deltaTime);
    }

    public void correct(SimpleMatrix measurement) {
        observer.correct(controller.getU(), measurement);
    }

    public SimpleMatrix getU() {
        return MatrixUtils.clamp(controller.getU(), plant.getuMin().get(0), plant.getuMax().get(0));
    }

    public double getError() {
        return controller.getR().minus(observer.getXhat()).get(0);
    }

    public Plant getPlant() {
        return plant;
    }

    public LinearQuadraticRegulator getController() {
        return controller;
    }

    public KalmanFilter getObserver() {
        return observer;
    }
}
