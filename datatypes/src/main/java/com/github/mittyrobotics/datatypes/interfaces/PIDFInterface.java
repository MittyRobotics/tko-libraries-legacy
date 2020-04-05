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

/**
 * Interface for setting up PIDF for various motor controllers
 */
public interface PIDFInterface {

    /**
     * Configures PIDF and the slot idx
     * Needs to be overridden
     *
     * @param p       the proportional value
     * @param i       the integral value
     * @param d       the derivative value
     * @param ff      the feed forward value
     * @param slotIdx the slotIdx for PIDF control to store different PIDF values
     */
    void configPIDF(double p, double i, double d, double ff, int slotIdx);

    /**
     * Configures PIDF on slot 0
     *
     * @param p  the proportional value
     * @param i  the integral value
     * @param d  the derivative value
     * @param ff the feed forward value
     */
    default void configPIDF(double p, double i, double d, double ff) {
        configPIDF(p, i, d, ff, 0);
    }

    /**
     * Configures PID on a slot idx (sets feed forward to 0)
     *
     * @param p       the proportional value
     * @param i       the integral value
     * @param d       the derivative value
     * @param slotIdx the slotIdx for PID control
     */
    default void configPID(double p, double i, double d, int slotIdx) {
        configPIDF(p, i, d, 0, slotIdx);
    }

    /**
     * Configures PID on slot 0 (sets feed forward to 0)
     *
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     */
    default void configPID(double p, double i, double d) {
        configPID(p, i, d, 0);
    }
}