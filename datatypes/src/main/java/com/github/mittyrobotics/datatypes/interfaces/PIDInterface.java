package com.github.mittyrobotics.datatypes.interfaces;

public interface PIDInterface {

    void configPIDF(double p, double i, double d, double ff, int slotIdx);

    default void configPIDF(double p, double i, double d, double ff){
        configPIDF(p, i, d, ff, 0);
    }

    default void configPID(double p, double i, double d, int slotIdx){
        configPIDF(p, i, d, 0, slotIdx);
    }

    default void configPID(double p, double i, double d){
        configPID(p, i, d, 0);
    }
}