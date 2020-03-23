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

package com.github.mittyrobotics.datatypes;

import java.util.ArrayList;
import java.util.Iterator;

public class TimestampedList<E> extends ArrayList<TimestampedElement<E>> {
    public void add(E object, double timestamp) {
        add(new TimestampedElement<E>(object, timestamp));
    }

    public void addFront(TimestampedElement<E> eTimestampedElement) {
        super.add(0, eTimestampedElement);
    }

    public E getElementFromTimestamp(double timestamp) {
        double closest = Double.POSITIVE_INFINITY;
        E object = null;
        Iterator<TimestampedElement<E>> iterator = iterator();
        while (iterator.hasNext()) {
            TimestampedElement<E> timestampedElement = iterator.next();
            double t = timestampedElement.getTimestamp();
            if (Math.abs(t - timestamp) < closest) {
                closest = Math.abs(t - timestamp);
                object = timestampedElement.getObject();
            }
        }
        return object;
    }

    public double getTimestampFromElement(int elementIndex) {
        return get(elementIndex).getTimestamp();
    }

    public TimestampedElement<E> getLatest() {
        return get(0);
    }
}
