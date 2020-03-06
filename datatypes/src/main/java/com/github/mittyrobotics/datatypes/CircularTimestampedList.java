/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

import java.util.Iterator;

public class CircularTimestampedList<E> {
    private final int circularSize;

    private TimestampedList<E> timestampedList = new TimestampedList<>();

    public CircularTimestampedList(int size){
        this.circularSize = size;
    }

    public int getCircularSize(){
        return circularSize;
    }

    public void addFront(TimestampedElement<E> timestampedElement) {
        timestampedList.addFront(timestampedElement);
        if(timestampedList.size() > circularSize){
            double oversize = timestampedList.size()-circularSize;
            for(int i = 0; i < oversize; i++){
                timestampedList.remove(timestampedList.size()-1);
            }
        }
    }

    public void setObject(int index, E object){
        if(timestampedList.get(index) != null){
            timestampedList.set(index,new TimestampedElement<>(object,get(index).getTimestamp()));
        }
    }

    public TimestampedElement<E> get(int index){
        //Cap index by the current list size and the circular size
        index = Math.min(size(),Math.min(circularSize,index));
        return timestampedList.get(index);
    }

    public TimestampedElement<E> getLatest(){
        return timestampedList.getLatest();
    }

    public TimestampedElement<E> getLast(){
        return timestampedList.get(timestampedList.size()-1);
    }

    public E getElementFromTimestamp(double timestamp){
        return timestampedList.getElementFromTimestamp(timestamp);
    }

    public double getTimestampFromElement(int elementIndex) {
        return timestampedList.getTimestampFromElement(elementIndex);
    }


    public int size(){
        return timestampedList.size();
    }
}
