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

package com.github.mittyrobotics.motion.statespace;

import org.ejml.simple.SimpleMatrix;

public class StateSpaceSystemGains {
    private SimpleMatrix a;
    private SimpleMatrix b;
    private SimpleMatrix c;
    private SimpleMatrix d;

    public StateSpaceSystemGains() {
    }

    public StateSpaceSystemGains(SimpleMatrix a, SimpleMatrix b, SimpleMatrix c, SimpleMatrix d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public SimpleMatrix getA() {
        return a;
    }

    public void setA(SimpleMatrix a) {
        this.a = a;
    }

    public SimpleMatrix getB() {
        return b;
    }

    public void setB(SimpleMatrix b) {
        this.b = b;
    }

    public SimpleMatrix getC() {
        return c;
    }

    public void setC(SimpleMatrix c) {
        this.c = c;
    }

    public SimpleMatrix getD() {
        return d;
    }

    public void setD(SimpleMatrix d) {
        this.d = d;
    }
}
