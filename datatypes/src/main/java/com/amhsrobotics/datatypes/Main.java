package com.amhsrobotics.datatypes;

import com.amhsrobotics.datatypes.geometry.Circle;
import com.amhsrobotics.datatypes.geometry.Line;
import com.amhsrobotics.datatypes.positioning.Position;
import com.amhsrobotics.datatypes.positioning.Transform;

public class Main {
	public static void main(String[] args) {
		Circle circle = new Circle(new Transform(10,10,34),new Position(5,5));
		System.out.println(circle.getCenter() + " " + circle.getRadius());
	}
}
