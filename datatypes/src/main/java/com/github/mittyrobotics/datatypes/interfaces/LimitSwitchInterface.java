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
 * Interface to add limit switches to motors that have been wired through the roborio instead of the motor controller
 */
public interface LimitSwitchInterface {

    /**
     * Configures the forward limit switch
     *
     * @param id        the id of the forward limit switch
     * @param inversion if the forward limit switch is inverted
     */
    void configRoborioForwardLimitSwitch(int id, boolean inversion);

    /**
     * Configures the reverse limit switch
     *
     * @param id        the id of the reverse limit switch
     * @param inversion if the reverse limit switch is inverted
     */
    void configRoborioReverseLimitSwitch(int id, boolean inversion);

    /**
     * Configures the forward limit switch
     *
     * @param id the id of the forward limit switch
     */
    default void configRoborioForwardLimitSwitch(int id) {
        configRoborioForwardLimitSwitch(id, false);
    }

    /**
     * Configures the reverse limit switch
     *
     * @param id the id of the reverse limit switch
     */
    default void configRoborioReverseLimitSwitch(int id) {
        configRoborioReverseLimitSwitch(id, false);
    }

    /**
     * Gets if the forward limit switch is triggered
     *
     * @return forward limit switch value
     */
    boolean getRoborioForwardLimitSwitch();

    /**
     * Gets if the reverse limit switch is triggered
     *
     * @return reverse limit switch value
     */
    boolean getRoborioReverseLimitSwitch();
}
