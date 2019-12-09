package com.github.mittyrobotics.datatypes;

import com.github.mittyrobotics.datatypes.geometry.ArcSegment;
import com.github.mittyrobotics.datatypes.positioning.Position;

public class Main {
	public static void main(String[] args) {
		ArcSegment arcSegment = new ArcSegment(new Position(5,0), new Position(0,5), new Position(Math.cos(Math.toRadians(1))*5,Math.sin(Math.toRadians(1))*5));
		System.out.println(arcSegment.isOnSegment(new Position(Math.cos(Math.toRadians(50))*5,Math.sin(Math.toRadians(50))*5)));
	}
}
