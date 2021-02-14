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

package com.github.mittyrobotics.motion.pathfollowing;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class Odometry {
    private Position deltaPosition;
    private Rotation robotRotation;
    private double lastLeftEncoder = 0;
    private double lastRightEncoder = 0;
    private double calibrateGyroVal = 0;

    private static Odometry instance;
    public static Odometry getInstance(){
        if(instance == null){
            instance = new Odometry();
        }
        return instance;
    }

    public void update(double leftEncoder, double rightEncoder, double gyro){
        //Get robot rotation
        robotRotation = Rotation.fromDegrees(gyro - calibrateGyroVal).mapDegrees180();

        //Get delta left and right encoder pos
        double deltaLeftPos = leftEncoder - lastLeftEncoder;
        double deltaRightPos = rightEncoder - lastRightEncoder;

        //Get average delta encoder pos in inches
        double deltaEncoder = (deltaLeftPos + deltaRightPos) / 2;

        //Get x and y values from heading and delta pos
        double deltaX = deltaEncoder * robotRotation.cos();
        double deltaY = deltaEncoder * robotRotation.sin();

        //Set last encoder positions
        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;

        //Get delta position
        deltaPosition = new Position(deltaX, deltaY);
    }

    public void zeroEncoders(double leftEncoder, double rightEncoder){
        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;
    }

    public void setHeading(double heading, double gyro) {
        calibrateGyroVal = gyro - heading;
    }

    public Position getDeltaPosition(){
        return deltaPosition;
    }

    public Rotation getRobotRotation(){
        return robotRotation;
    }
}
