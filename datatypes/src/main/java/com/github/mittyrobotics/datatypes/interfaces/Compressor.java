package com.github.mittyrobotics.datatypes.interfaces;

public class Compressor extends edu.wpi.first.wpilibj.Compressor implements  IHardware {

    private static Compressor instance;

    public static Compressor getInstance(){
        if(instance == null){
            instance = new Compressor();
        }
        return instance;
    }

    private Compressor(){
        super();
    }

    @Override
    public void initHardware() {
        super.setClosedLoopControl(true);
    }
}
