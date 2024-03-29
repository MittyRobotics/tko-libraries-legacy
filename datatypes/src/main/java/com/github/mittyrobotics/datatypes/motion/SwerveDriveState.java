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

package com.github.mittyrobotics.datatypes.motion;

public class SwerveDriveState {
    private SwerveModuleState frState = new SwerveModuleState(0, 0);
    private SwerveModuleState flState = new SwerveModuleState(0, 0);
    private SwerveModuleState blState = new SwerveModuleState(0, 0);
    private SwerveModuleState brState = new SwerveModuleState(0, 0);
    private double velX;
    private double velY;
    private double angularVel;

    public SwerveDriveState() {
        this(0, 0, 0, new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState());
    }

    public SwerveDriveState(double velX, double velY, double angularVel, SwerveModuleState frState,
                            SwerveModuleState flState, SwerveModuleState blState, SwerveModuleState brState) {
        this.velX = velX;
        this.velY = velY;
        this.angularVel = angularVel;
        this.frState = frState;
        this.flState = flState;
        this.blState = blState;
        this.brState = brState;
    }

    public SwerveDriveState(double velX, double velY, double angularVel) {
        this(velX, velY, angularVel, new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState());
    }

    public SwerveDriveState(SwerveModuleState frState, SwerveModuleState flState, SwerveModuleState blState,
                            SwerveModuleState brState) {
        this(0, 0, 0, frState, flState, blState, brState);
    }

    public static SwerveDriveState solveFromInverseKinematics(double length, double width, double velX, double velY,
                                                              double angularVel) {
        return SwerveDriveKinematics.solveInverseKinematics(length, width, velX, velY, angularVel);
    }

    public static SwerveDriveState solveFromForwardKinematics(double length, double width, SwerveModuleState frState,
                                                              SwerveModuleState flState, SwerveModuleState blState,
                                                              SwerveModuleState brState) {
        return SwerveDriveKinematics.solveForwardKinematics(length, width, frState, flState, blState, brState);
    }

    public double getVelX() {
        return velX;
    }

    public void setVelX(double velX) {
        this.velX = velX;
    }

    public double getVelY() {
        return velY;
    }

    public void setVelY(double velY) {
        this.velY = velY;
    }

    public double getAngularVel() {
        return angularVel;
    }

    public void setAngularVel(double angularVel) {
        this.angularVel = angularVel;
    }

    @Override
    public String toString() {
        return "SwerveDriveState{" +
                "velX=" + velX +
                ", velY=" + velY +
                ", angularVel=" + angularVel +
                '}';
    }

    public SwerveModuleState getFrState() {
        return frState;
    }

    public void setFrState(SwerveModuleState frState) {
        this.frState = frState;
    }

    public SwerveModuleState getFlState() {
        return flState;
    }

    public void setFlState(SwerveModuleState flState) {
        this.flState = flState;
    }

    public SwerveModuleState getBlState() {
        return blState;
    }

    public void setBlState(SwerveModuleState blState) {
        this.blState = blState;
    }

    public SwerveModuleState getBrState() {
        return brState;
    }

    public void setBrState(SwerveModuleState brState) {
        this.brState = brState;
    }
}
