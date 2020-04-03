/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Enchanced {@link DoubleSolenoid} with extra features
 */
public class TKODoubleSolenoid extends DoubleSolenoid implements InversionInterface {

    /**
     * If the double solenoid is inverted or not
     */
    private boolean isInverted;

    /**
     * Constructs a new Double Solenoid
     * @param forwardChannel the forward channel port plugged into the PCM
     * @param reverseChannel the reverse channel port plugged into the PCM
     * @param pcmID the ID of the PCM
     */
    public TKODoubleSolenoid(int forwardChannel, int reverseChannel, int pcmID) {
        super(pcmID, forwardChannel, reverseChannel);
        setInverted(false);
    }

    /**
     * Constructs a new Double Solenoid
     * Sets the PCM ID to 0
     * @param forwardChannel the forward channel port plugged into the PCM
     * @param reverseChannel the reverse channel port plugged into the PCM
     */
    public TKODoubleSolenoid(int forwardChannel, int reverseChannel) {
        this(forwardChannel, reverseChannel, 0);
    }

    /**
     * Sets the value of the double solenoid
     * Flips forward and reverse if the double solenoid is inverted
     * @param value the value to set the double solenoid to
     */
    @Override
    public void set(Value value) {
        if (getInverted()) {
            if (value == Value.kReverse) {
                value = Value.kForward;
            } else if (value == Value.kForward) {
                value = Value.kReverse;
            }
        }
        super.set(value);
    }

    /**
     * Returns if the double solenoid is inverted
     * @return isInverted
     */
    @Override
    public boolean getInverted() {
        return isInverted;
    }

    /**
     * Sets the inversion of the double solenoid
     * @param inversion the value to set the double solenoid inversion to
     */
    @Override
    public void setInverted(boolean inversion) {
        isInverted = inversion;
    }
}