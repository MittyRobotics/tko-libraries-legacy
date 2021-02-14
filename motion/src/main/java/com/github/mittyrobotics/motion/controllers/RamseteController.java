///*
// *  MIT License
// *
// *  Copyright (c) 2020 Mitty Robotics (Team 1351)
// *
// *  Permission is hereby granted, free of charge, to any person obtaining a copy
// *  of this software and associated documentation files (the "Software"), to deal
// *  in the Software without restriction, including without limitation the rights
// *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// *  copies of the Software, and to permit persons to whom the Software is
// *  furnished to do so, subject to the following conditions:
// *
// *  The above copyright notice and this permission notice shall be included in all
// *  copies or substantial portions of the Software.
// *
// *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// *  SOFTWARE.
// */
//
//package com.github.mittyrobotics.motion.controllers;
//
//import com.github.mittyrobotics.datatypes.motion.DrivetrainState;
//import com.github.mittyrobotics.datatypes.positioning.Rotation;
//import com.github.mittyrobotics.datatypes.positioning.Transform;
//import com.github.mittyrobotics.datatypes.positioning.TransformWithParameter;
//import com.github.mittyrobotics.motion.pathfollowing.PathFollower;
//import com.github.mittyrobotics.motion.pathfollowing.PathFollowerProperties;
//
//public class RamseteController extends PathFollower {
//    public static double DEFAULT_AGGRESSIVE_GAIN = 2.0;
//    public static double DEFAULT_DAMPING_GAIN = 0.2;
//
//    private PathFollowerProperties.RamseteProperties ramseteProperties;
//
//    public RamseteController(PathFollowerProperties properties,
//                             PathFollowerProperties.RamseteProperties ramseteProperties) {
//        super(properties);
//        this.ramseteProperties = ramseteProperties;
//    }
//
//    @Override
//    public DrivetrainState calculate(Transform robotTransform, DrivetrainState currentDrivetrainVelocities,
//                                     double deltaTime) {
//        //Get the desired transform to follow, which is the closest point on the path
//        TransformWithParameter desiredTransform = getCurrentPath().getClosestTransform(robotTransform.getPosition());
//
//        //If reversed, reverse the desired transform's rotation
//        desiredTransform
//                .setRotation(
//                        desiredTransform.getRotation().rotateBy(Rotation.fromDegrees((getProperties().reversed ? 180 :
//                                0))));
//
//        //Calculate the robot velocity using the path velocity controller. If reversed, reverse the robot velocity
//        double robotVelocity = getProperties().velocityController
//                .getVelocity(getPreviousCalculatedVelocity(), getTraveledDistance(),
//                        deltaTime);
//
//        setPreviousCalculatedVelocity(robotVelocity);
//
//        //Get radius from curvature is 1/curvature
//        double turningRadius = 1 / getCurrentPath().getCurvature(desiredTransform.getParameter());
//
//        if (Double.isNaN(turningRadius) || Double.isInfinite(turningRadius)) {
//            turningRadius = 2e16;
//        }
//
//        DrivetrainState velocity =
//                DrivetrainState.fromLinearAndRadius(robotVelocity, turningRadius, getProperties().trackWidth);
//
//        //Get the transform error in meters.
//        Transform error = desiredTransform.relativeTo(robotTransform);
//
//        double eX = error.getPosition().getX();
//        double eY = error.getPosition().getY();
//        double eTheta = error.getRotation().getRadians();
//
//        //Calculate the Ramsete k value
//        double k = 2.0 * ramseteProperties.dampingGain *
//                Math.sqrt(Math.pow(velocity.getAngular(), 2) +
//                        ramseteProperties.aggressiveGain * Math.pow(velocity.getLinear(), 2));
//
//        //Calculate the adjusted linear velocity from the Ramsete algorithm
//        double adjustedLinearVelocity = velocity.getLinear() * error.getRotation().cos() + k * eX;
//
//        //Calculate the adjusted angular velocity from the Ramsete algorithm (stays in radians per second)
//        double adjustedAngularVelocity =
//                velocity.getAngular() + k * eTheta +
//                        ramseteProperties.aggressiveGain * velocity.getLinear() * error.getRotation().sinc() * eY;
//
//        //Calculate drivetrain state from linear and angular velocity
//        DrivetrainState state = DrivetrainState
//                .fromLinearAndAngular(adjustedLinearVelocity,
//                        adjustedAngularVelocity * (getProperties().reversed ? -1 : 1),
//                        velocity.getTrackWidth());
//
//        //Reversed controller
//        if (getProperties().reversed) {
//            state = state.reverse();
//        }
//
//        return state;
//    }
//}
