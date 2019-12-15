package com.github.mittyrobotics.simulation;

public class SimMain {
    public static void main(String[] args) {
        SimSampleRobot robot = new SimSampleRobot();
        RobotSimManager.getInstance().setupRobotSimManager(robot,SimSampleDrivetrain.getInstance(), 125,7,2,20,30,0.02);
        SimOI.getInstance();
    }

}

