package com.amhsrobotics.datatypes;

import com.amhsrobotics.datatypes.geometry.Arc;
import com.amhsrobotics.datatypes.geometry.Circle;
import com.amhsrobotics.datatypes.geometry.Line;
import com.amhsrobotics.datatypes.positioning.Position;
import com.amhsrobotics.datatypes.positioning.Rotation;
import com.amhsrobotics.datatypes.positioning.Transform;

public class Main {
	public static void main(String[] args) {
		Arc arc = new Arc(new Circle(new Position(0,0),5),new Position(5,0), new Position(0,5), new Position(Math.cos(Math.toRadians(1))*5,Math.sin(Math.toRadians(1))*5));
		System.out.println(arc.isOnArc(new Position(Math.cos(Math.toRadians(89))*5,Math.sin(Math.toRadians(89))*5)));
	}
}
