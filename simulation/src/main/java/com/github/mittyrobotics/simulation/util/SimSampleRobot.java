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

package com.github.mittyrobotics.simulation.util;

import com.github.mittyrobotics.simulation.sim.SimRobot;

public class SimSampleRobot implements SimRobot {

    @Override
    public void robotInit() {


    }

    @Override
    public void robotPeriodic() {
        //arcadeDrive();
//		System.out.println(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity() + " "  + SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
    }

    private void arcadeDrive() {
        double drive;
        double turn;
        if (SimOI.getInstance().isUpKey()) {
            drive = 0.5;
        } else if (SimOI.getInstance().isDownKey()) {
            drive = -0.5;
        } else {
            drive = 0;
        }
        if (SimOI.getInstance().isLeftKey()) {
            turn = -0.2;
        } else if (SimOI.getInstance().isRightKey()) {
            turn = 0.2;
        } else {
            turn = 0;
        }
        SimSampleDrivetrain.getInstance().setSpeeds(drive + turn, drive - turn);
    }
}
