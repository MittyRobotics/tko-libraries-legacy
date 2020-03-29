package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.DigitalInput;

public class TKODigitalInput extends DigitalInput {

    private boolean isInverted;

    public TKODigitalInput(int channel) {
        super(channel);
    }

    public void setInverted(boolean inversion) {
        isInverted = inversion;
    }

    @Override
    public boolean get() {
        return isInverted != super.get();
    }
}