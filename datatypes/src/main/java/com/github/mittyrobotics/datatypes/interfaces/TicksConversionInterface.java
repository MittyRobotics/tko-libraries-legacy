package com.github.mittyrobotics.datatypes.interfaces;

public interface TicksConversionInterface {
    void setTicksToUnit(double ticksPerUnit);
    double getPositionRaw();
    double getVelocityRaw();
    double getPosition();
    double getVelocity();
}