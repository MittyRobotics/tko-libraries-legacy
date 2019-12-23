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

package com.github.mittyrobotics.motionprofile.util;

/**
 * https://gist.github.com/JoseRivas1998/f6642e1e8dcea665b12e0f7264d3e088#file-mainwithoutlambda-java
 */
public class IntegralMath {
	public static final double INCREMENT = 1E-4;
	
	public static void main(String[] args) {
		System.out.println(integral(0, 2, new Function() {
			@Override
			public double f(double x) {
				return Math.pow(x, 2);
			}
		}));
	}
	
	public static double integral(double a, double b, Function function) {
		double area = 0;
		double modifier = 1;
		if (a > b) {
			double tempA = a;
			a = b;
			b = tempA;
			modifier = -1;
		}
		for (double i = a + INCREMENT; i < b; i += INCREMENT) {
			double dFromA = i - a;
			area += (INCREMENT / 2) * (function.f(a + dFromA) + function.f(a + dFromA - INCREMENT));
		}
		return (Math.round(area * 1000.0) / 1000.0) * modifier;
	}
}
