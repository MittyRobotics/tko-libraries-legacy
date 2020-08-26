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

package com.github.mittyrobotics.motion;

import com.github.mittyrobotics.datatypes.units.Conversions;
import com.github.mittyrobotics.motion.statespace.models.PulleyModel;
import com.github.mittyrobotics.motion.statespace.motors.Pro775Motor;
import com.github.mittyrobotics.visualization.MotorGraph;

public class ModelDemonstration {
    public static void main(String[] args) throws InterruptedException {
        PulleyModel model =
                new PulleyModel(new Pro775Motor(2), 10 * Conversions.LBS_TO_KG, 7, 2 * Conversions.IN_TO_M, 12);

        MotorGraph graph = new MotorGraph();

//        PIDFController controller = new PIDFController(.08, 0, 0);
//        controller.setSetpoint(10);

        double dt = 0.02;
        for (double t = 0; t < 10; t += dt) {
//            double voltage = controller.calculate(model.getPosition(), dt) * 12;
            double voltage = 5;

            model.updateModel(voltage, dt);

            graph.addMotorValues(model.getPosition(), model.getVelocity(), model.getAcceleration(), voltage, t);

            Thread.sleep((long) (dt * 1000));
        }
    }
}
