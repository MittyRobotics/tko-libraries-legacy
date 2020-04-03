package com.github.mittyrobotics.datatypes.interfaces;

public interface TicksConversionInterface {
    void setTicksPerUnit(double ticksPerUnit);
    double getPositionRaw();
    double getVelocityRaw();
    double getPosition();
    double getVelocity();
    double getTicksPerUnit();
}