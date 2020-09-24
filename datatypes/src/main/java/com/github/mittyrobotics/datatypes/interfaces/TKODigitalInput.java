///*
// * MIT License
// *
// * Copyright (c) 2020 Mitty Robotics (Team 1351)
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// *
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package com.github.mittyrobotics.datatypes.interfaces;
//
//import edu.wpi.first.wpilibj.DigitalInput;
//
///**
// * Enhanced {@link DigitalInput} with extra features for switches wired through the roborio
// */
//public class TKODigitalInput extends DigitalInput implements InversionInterface {
//
//    /**
//     * If the switch is inverted or not
//     */
//    private boolean isInverted;
//
//    /**
//     * Constructs a new {@link TKODigitalInput} object
//     *
//     * @param channel the port the switch is wired to on the roborio
//     */
//    public TKODigitalInput(int channel) {
//        super(channel);
//    }
//
//    /**
//     * Returns if the switch is triggered
//     *
//     * @return the raw {@link DigitalInput} value xor isInverted
//     */
//    @Override
//    public boolean get() {
//        return getInverted() != super.get();
//    }
//
//    /**
//     * Returns if the switch is inverted
//     *
//     * @return isInverted
//     */
//    @Override
//    public boolean getInverted() {
//        return isInverted;
//    }
//
//    /**
//     * Sets the switch to be inverted
//     *
//     * @param inversion the value to set the switch inversion to
//     */
//    @Override
//    public void setInverted(boolean inversion) {
//        isInverted = inversion;
//    }
//}