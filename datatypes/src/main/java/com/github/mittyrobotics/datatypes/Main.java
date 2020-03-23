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

public class Main {
    public static void main(String[] args) {
        CircularTimestampedList<Integer> list = new CircularTimestampedList<>(2);
        list.addFront(new TimestampedElement<>(0, 0));
        list.addFront(new TimestampedElement<>(1, 1));
        list.addFront(new TimestampedElement<>(2, 2));
        list.addFront(new TimestampedElement<>(3, 3));
        list.addFront(new TimestampedElement<>(4, 4));
        list.addFront(new TimestampedElement<>(5, 5));
        list.addFront(new TimestampedElement<>(6, 6));
        list.addFront(new TimestampedElement<>(7, 7));
        list.addFront(new TimestampedElement<>(8, 8));
        list.addFront(new TimestampedElement<>(9, 9));
        for (int i = 0; i < list.size(); i++) {
            System.out.println(list.get(i).getObject() + " " + list.get(i).getTimestamp());
        }
    }
}
