package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.DigitalInput;

public class TKODigitalInput extends DigitalInput implements InversionInterface {

    public TKODigitalInput(int channel) {
        super(channel);
    }

    @Override
    public boolean get() {
        return getInverted() != super.get();
    }
}