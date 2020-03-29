package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class TKODoubleSolenoid extends DoubleSolenoid implements InversionInterface {
    public TKODoubleSolenoid(int forwardChannel, int reverseChannel, int pcmID) {
        super(pcmID, forwardChannel, reverseChannel);
        setInverted(false);
    }

    public TKODoubleSolenoid(int forwardChannel, int reverseChannel){
        this(0, forwardChannel, reverseChannel);
    }

    @Override
    public void set(Value value) {
        if(getInverted()){
            if(value == Value.kReverse){
                value = Value.kForward;
            } else if(value == Value.kForward){
                value = Value.kReverse;
            }
        }
        super.set(value);
    }
}