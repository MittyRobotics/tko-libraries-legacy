package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro extends ADXRS450_Gyro implements IHardware{
    private static Gyro instance;

    public static Gyro getInstance(){
        if(instance == null){
            instance = new Gyro();
        }
        return instance;
    }

    private Gyro(){
        super();
    }

    @Override
    public void initHardware() {
        super.reset();
        super.calibrate();
    }
}