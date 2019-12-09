package com.github.mittyrobotics;

import com.github.mittyrobotics.geometry.Arc;
import com.github.mittyrobotics.geometry.Circle;
import com.github.mittyrobotics.positioning.Position;

public class Main {
	public static void main(String[] args) {
		Arc arc = new Arc(new Circle(new Position(0,0),5),new Position(5,0), new Position(0,5), new Position(Math.cos(Math.toRadians(1))*5,Math.sin(Math.toRadians(1))*5));
		System.out.println(arc.isOnArc(new Position(Math.cos(Math.toRadians(89))*5,Math.sin(Math.toRadians(89))*5)));
	}
}
