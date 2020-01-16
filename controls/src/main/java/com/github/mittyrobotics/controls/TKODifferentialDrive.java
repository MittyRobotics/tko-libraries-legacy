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

package com.github.mittyrobotics.controls;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

public class TKODifferentialDrive {
    SpeedController leftController;
    SpeedController rightController;
    
    public TKODifferentialDrive(SpeedController leftController, SpeedController rightController) {
        this.leftController = leftController;
        this.rightController = rightController;
    }

    public void tankDrive(double left, double right) {
        if (Math.abs(left) < 0.1) {
            leftController.set(0);
        } else {
            leftController.set(left);
        }
        if (Math.abs(right) < 0.1) {
            rightController.set(0);
        } else {
            rightController.set(right);
        }
    }

    public void carDriveCarSteering(double turn, boolean isLeftPressed, boolean isRightPressed, boolean brake, double customAcceleration, double DRIVE_EMPHASIS, double deadZone) {
        double speed = 0;
        double acceleration = 0;


        if(brake){
            speed = 0;
            acceleration = 0;
            turn = 0;
        } else if(speed < 0) { //if going backward
            //make if left pressed, right pressed and if nothing pressed
            if(isLeftPressed){ //left bumper
                if(speed > -1) { //capped speed
                    acceleration = -customAcceleration;
                } else {
                    acceleration = 0;
                }
            } else if (isRightPressed){ //right bumper
                acceleration = customAcceleration;
            } else { //let go both bumpers
                if(speed < 0){
                    acceleration = customAcceleration;
                } else {
                    acceleration = 0;
                }
            }
        } else if (speed == 0) { //robot at rest
            //make if left pressed, right pressed and if nothing pressed
            if (isLeftPressed){ //left bumper
                acceleration = -customAcceleration;
            }
            else if (isRightPressed){ //right bumper
                acceleration = customAcceleration;
            }
            else { //let go both
                acceleration = 0;
            }
        } else if(speed > 0) { //going forward
            //make if left pressed, right pressed and if nothing pressed
            if (isLeftPressed) { //left bumper
                acceleration = -customAcceleration;
            } else if (isRightPressed) { //right bumper
                if (speed < 1) { //capped speed
                    acceleration = customAcceleration;
                } else {
                    acceleration = 0;
                }
            } else { //let go both
                if (speed > 0) {
                    acceleration = -customAcceleration;

                } else {
                    acceleration = 0;
                }
            }
        }

        speed += acceleration;

        double newSpeed = speed * DRIVE_EMPHASIS;
        double newTurn = turn * (1-DRIVE_EMPHASIS);
        if(Math.abs(speed) < deadZone && !isLeftPressed && !isRightPressed){
            tankDrive(turn, -turn);
        }

        if(speed != 0) {
            tankDrive(newSpeed + newTurn, newSpeed - newTurn);
        }

    }

    public void carDriveCarSteering(double turn, boolean isLeftPressed, boolean isRightPressed, boolean brake, int step) {
        carDriveCarSteering(turn, isLeftPressed, isRightPressed, brake, 0.07, 0.3, 0.1);
    }

    public void carDriveCompassSteering(double p, double i, double d, double steerWheelValue, int step, double gyroAngle, boolean isLeftPressed, boolean isRightPressed, boolean brake, double DRIVE_EMPHASIS, double deadZone, double customAcceleration) {
        double speed = 0;
        double acceleration = 0;
        double turn = 0;

        PIDController controller = new PIDController(p, i, d);
        controller.setIntegratorRange(1, -1);


        //CompassSteering
        steerWheelValue = steerWheelValue * 450;

        if(steerWheelValue > gyroAngle + step) {
            controller.setSetpoint(gyroAngle + step);
        } else if (steerWheelValue < gyroAngle - step) {
            controller.setSetpoint(gyroAngle - step);
        } else {
            controller.setSetpoint(steerWheelValue);
        }

        //BumperDriveCommand
        if(speed < 0){ //if going backward
            //make if left pressed, right pressed and if nothing pressed
            if (isLeftPressed){ //left bumper
                if(speed > -1){ //capped speed
                    acceleration = -customAcceleration;
                } else {
                    acceleration = 0;
                }
            }
            else if (isRightPressed){ //right bumper
                acceleration = customAcceleration;
            }
            else { //let go both bumpers
                if (speed < 0){
                    acceleration = customAcceleration;
                } else {
                    acceleration = 0;
                }
            }
        }

        else if (speed == 0){ //robot at rest
            //make if left pressed, right pressed and if nothing pressed
            if (isLeftPressed){ //left bumper
                acceleration = -customAcceleration;
            }
            else if (isRightPressed){ //right bumper
                acceleration = customAcceleration;
            }
            else { //let go both
                acceleration = 0;
            }
        }

        else if(speed > 0){ //going forward
            //make if left pressed, right pressed and if nothing pressed
            if (isLeftPressed){ //left bumper
                acceleration = -customAcceleration;
            }
            else if (isRightPressed){ //right bumper
                if (speed < 1){ //capped speed
                    acceleration = customAcceleration;
                } else {
                    acceleration = 0;
                }
            }
            else { //let go both
                if (speed > 0) {
                    acceleration = -customAcceleration;

                } else {
                    acceleration = 0;
                }
            }
        }

        speed += acceleration;

        turn = controller.getSetpoint();

        //emphasis var
        double newSpeed = speed * DRIVE_EMPHASIS;
        double newTurn = turn * (1 - DRIVE_EMPHASIS);

        if(brake){
            tankDrive(0, 0);
        } else if(Math.abs(speed) < deadZone){
            tankDrive(turn, -turn);
        } else if (speed > 0) {
            tankDrive((newSpeed) - newTurn, (newSpeed) + newTurn);
        } else {
            tankDrive((newSpeed) + newTurn, (newSpeed) - newTurn);
        }

    }

    public void carDriveCompassSteering(double steerWheelValue, double gyroAngle, boolean isLeftPressed, boolean isRightPressed, boolean brake) {
        carDriveCompassSteering(0.03, 0, 0.035, steerWheelValue, 15, gyroAngle, isLeftPressed, isRightPressed, brake, 0.7, 0.1, 0.7);
    }

    public void joystickCarSteering(double steeringWheelX, double joystickY, boolean joystickTrigger, double deadZone){
        double turn = steeringWheelX * 450 / 120;
        double speed = -joystickY;
        boolean brake = joystickTrigger;

        if(Math.abs(turn) > 1){
            turn = Math.signum(turn);
        }
        double e = 1 - turn;

        if(brake){
            speed = 0;
            turn = 0;
        }

        double newSpeed = speed*e;
        double newTurn = turn;

        if(Math.abs(speed) < deadZone){
            tankDrive(newTurn, - newTurn);
        }
        else if(speed >= 0){
            tankDrive(newSpeed + newTurn, newSpeed - newTurn);
        } else {
            tankDrive(newSpeed - newTurn, newSpeed + newTurn);
        }

    }

    public void joystickCarSteering(double steeringWheelX, double joystickY, boolean joystickTrigger){
        joystickCarSteering(steeringWheelX, joystickY, joystickTrigger, 0.05);
    }

    public void joystickCompassSteering(double p, double i, double d, double joystickY, double steeringWheelX, int step, double gyroAngle){
        PIDController controller = new PIDController(p, i, d);
        controller.setIntegratorRange(1, -1);
        double speed = -joystickY;
        double steerWheelValue = steeringWheelX * 450;

        if (steerWheelValue > gyroAngle + step) {
            controller.setSetpoint(gyroAngle + step);
        }
        else if (steerWheelValue < gyroAngle - step) {
            controller.setSetpoint(gyroAngle - step);
        }
        else {
            controller.setSetpoint(steerWheelValue);
        }
        double newSpeed = speed * (1-controller.getSetpoint());
        double newTurn = controller.getSetpoint();

        tankDrive(newSpeed + newTurn, newSpeed - newTurn);

    }

    public void joystickCompassSteering(double joystickY, double steeringWheelX, double gyroAngle) {
        joystickCompassSteering(0.03, 0, 0.035, joystickY, steeringWheelX, 15, gyroAngle);
    }

    public void pedalCarSteering(double steeringWheelX, double gas, double brake, boolean BButton, double DRIVE_EMPHASIS) {
        double turn = steeringWheelX * 450 / 180;
        double speed = 1 - gas;

        if ((brake > 0) && (brake < 1)) {
            speed = 0;
            turn = 0;
        }

        if(BButton){
            speed *= -1;
        }

        if (turn > 1) {
            turn = 1;
        }
        if (turn < -1) {
            turn = -1;
        }

        double newSpeed = speed* DRIVE_EMPHASIS;
        double newTurn = turn * (1- DRIVE_EMPHASIS);

        if(Math.abs(speed) < 0.1){
            tankDrive(turn, -turn);
        }

        else if (speed > 0) {
            tankDrive((newSpeed) + newTurn, (newSpeed) - newTurn);
        }
        else if (speed < 0) {
            tankDrive((newSpeed) - newTurn, (newSpeed) + newTurn);
        }

    }


    public void pedalCarSteering(double steeringWheelX, double gas, double brake, boolean BButton) {
        pedalCarSteering(steeringWheelX, gas, brake, BButton, 0.5);
    }

}
