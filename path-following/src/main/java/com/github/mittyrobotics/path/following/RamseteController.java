package com.github.mittyrobotics.path.following;

public class RamseteController {
	private static RamseteController instance = new RamseteController();
	
	public static RamseteController getInstance() {
		return instance;
	}
	
	private RamseteController(){
	
	}
}
