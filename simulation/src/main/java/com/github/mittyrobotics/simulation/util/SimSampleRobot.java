package com.github.mittyrobotics.simulation.util;

import com.github.mittyrobotics.simulation.sim.SimRobot;

public class SimSampleRobot implements SimRobot {
	
	@Override
	public void robotInit() {


	}
	
	@Override
	public void robotPeriodic() {
		//arcadeDrive();
//		System.out.println(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity() + " "  + SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
	}
	
	private void arcadeDrive() {
		double drive;
		double turn;
		if (SimOI.getInstance().isUpKey()) {
			drive = 0.5;
		} else if (SimOI.getInstance().isDownKey()) {
			drive = -0.5;
		} else {
			drive = 0;
		}
		if (SimOI.getInstance().isLeftKey()) {
			turn = -0.2;
		} else if (SimOI.getInstance().isRightKey()) {
			turn = 0.2;
		} else {
			turn = 0;
		}
		SimSampleDrivetrain.getInstance().setSpeeds(drive + turn, drive - turn);
	}
}
