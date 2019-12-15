package com.github.mittyrobotics.simulation;
import com.github.mittyrobotics.simulation.motors.CIMMotor;

public class SimSampleDrivetrain extends SimDrivetrain {
	private static SimSampleDrivetrain instance = new SimSampleDrivetrain();
	
	public static SimSampleDrivetrain getInstance() {
		return instance;
	}
	
	
	@Override
	void initDrivetrain() {
		SimTalon[] leftTalons = new SimTalon[]{
				new SimTalon(new ModelSystem(new CIMMotor())),
				new SimTalon(new ModelSystem(new CIMMotor()))
		};
		SimTalon[] rightTalons = new SimTalon[]{
				new SimTalon(new ModelSystem(new CIMMotor())),
				new SimTalon(new ModelSystem(new CIMMotor()))
		};
		
		setupSimDriveTalons(leftTalons,rightTalons);
		
		leftTalons[1].setFollower(leftTalons[0]);
		rightTalons[1].setFollower(rightTalons[0]);
	}
	
	public void setSpeeds(double left, double right){
		getLeftMasterTalon().set(left);
		getRightMasterTalon().set(right);
	}
}
