package com.github.mittyrobotics.datatypes;

import com.github.mittyrobotics.datatypes.geometry.Circle;
import com.github.mittyrobotics.datatypes.positioning.Position;

public class Main {
	public static void main(String[] args) {
		Position point = new Circle(new Position(5, 5), 6).getClosestPoint(new Position(15, 13));
		System.out.println(point);
	}
}
