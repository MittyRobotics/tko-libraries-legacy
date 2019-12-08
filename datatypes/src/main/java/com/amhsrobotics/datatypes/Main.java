package com.amhsrobotics.datatypes;

import com.amhsrobotics.datatypes.geometry.Circle;
import com.amhsrobotics.datatypes.geometry.Line;
import com.amhsrobotics.datatypes.positioning.Position;

public class Main {
	public static void main(String[] args) {
		Circle circle1 = new Circle(new Position(0,0),10);
		Line line = new Line(1,5);
		Position[] positions = circle1.circleLineIntersection(line);
		
		System.out.println(positions[0].toString() + " " + positions[1].toString() );
	}
}
