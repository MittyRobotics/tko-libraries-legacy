package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.DigitalInput;

public class TKODigitalInput extends DigitalInput implements InversionInterface {

    private boolean isInverted;

    public TKODigitalInput(int channel) {
        super(channel);
    }

    @Override
    public boolean get() {
        return getInverted() != super.get();
    }

    @Override
    public void setInverted(boolean inversion) {
        isInverted = inversion;
    }

    @Override
    public boolean getInverted() {
        return isInverted;
    }
}