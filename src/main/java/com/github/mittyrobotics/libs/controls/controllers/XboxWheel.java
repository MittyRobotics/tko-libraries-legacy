package com.github.mittyrobotics.libs.controls.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class XboxWheel extends GenericHID {
	public XboxWheel(int port) {
		super(port);
	}

	private enum Buttons {
		A(1),
		B(2),
		X(3),
		Y(4),
		LB(5),
		RB(6),
		LSB(7),
		RSB(8),
		Start(9),
		Select(10);

		int value;

		Buttons(int i) {
			value = i;
		}
	}

	private enum Axises {
		Wheel(0),
		Gas(1),
		Brake(2),
		Clutch(3);

		int value;

		Axises(int i) {
			value = i;
		}
	}

	@Override
	public double getX(Hand hand) {
		return getRawAxis(Axises.Wheel.value);
	}

	@Override
	public double getY(Hand hand) {
		return 0;
	}

	public double getGas(){
		return getRawAxis(Axises.Gas.value);
	}

	public double getBrake(){
		return getRawAxis(Axises.Brake.value);
	}

	public double getClutch(){
		return getRawAxis(Axises.Clutch.value);
	}

	public boolean getAButton(){
		return getRawButton(Buttons.A.value);
	}

	public boolean getAButtonPressed(){
		return getRawButtonPressed(Buttons.A.value);
	}

	public boolean getAButtonReleased(){
		return getRawButtonReleased(Buttons.A.value);
	}

	public boolean getBButton(){
		return getRawButton(Buttons.B.value);
	}

	public boolean getBButtonPressed(){
		return getRawButtonPressed(Buttons.B.value);
	}

	public boolean getBButtonReleased(){
		return getRawButtonReleased(Buttons.B.value);
	}

	public boolean getXButton(){
		return getRawButton(Buttons.X.value);
	}

	public boolean getXButtonPressed(){
		return getRawButtonPressed(Buttons.X.value);
	}

	public boolean getXButtonReleased(){
		return getRawButtonReleased(Buttons.X.value);
	}

	public boolean getYButton(){
		return getRawButton(Buttons.Y.value);
	}

	public boolean getYButtonPressed(){
		return getRawButtonPressed(Buttons.Y.value);
	}

	public boolean getYButtonReleased(){
		return getRawButtonReleased(Buttons.Y.value);
	}

	public boolean getShifter(Hand hand){
		switch (hand){
			case kLeft:
				return getRawButton(Buttons.LB.value);
			case kRight:
				return getRawButton(Buttons.RB.value);
		}
		return getRawButton(Buttons.LB.value);
	}

	public boolean getShifterPressed(Hand hand){
		switch (hand){
			case kLeft:
				return getRawButtonPressed(Buttons.LB.value);
			case kRight:
				return getRawButtonPressed(Buttons.RB.value);
		}
		return getRawButtonPressed(Buttons.LB.value);
	}

	public boolean getShifterReleased(Hand hand){
		switch (hand){
			case kLeft:
				return getRawButtonReleased(Buttons.LB.value);
			case kRight:
				return getRawButtonReleased(Buttons.RB.value);
		}
		return getRawButtonReleased(Buttons.LB.value);
	}

	public boolean getBumper(Hand hand){
		switch (hand){
			case kLeft:
				return getRawButton(Buttons.LSB.value);
			case kRight:
				return getRawButton(Buttons.RSB.value);
		}
		return getRawButton(Buttons.LSB.value);
	}

	public boolean getBumperPressed(Hand hand){
		switch (hand){
			case kLeft:
				return getRawButtonPressed(Buttons.LSB.value);
			case kRight:
				return getRawButtonPressed(Buttons.RSB.value);
		}
		return getRawButtonPressed(Buttons.LSB.value);
	}

	public boolean getBumperReleased(Hand hand){
		switch (hand){
			case kLeft:
				return getRawButtonReleased(Buttons.LSB.value);
			case kRight:
				return getRawButtonReleased(Buttons.RSB.value);
		}
		return getRawButtonReleased(Buttons.LSB.value);
	}

	public boolean getStartButton(){
		return getRawButton(Buttons.Start.value);
	}

	public boolean getStartButtonPressed(){
		return getRawButtonPressed(Buttons.Start.value);
	}

	public boolean getStartButtonReleased(){
		return getRawButtonReleased(Buttons.Start.value);
	}

	public boolean getSelectButton(){
		return getRawButton(Buttons.Select.value);
	}

	public boolean getSelectButtonPressed(){
		return getRawButtonPressed(Buttons.Select.value);
	}

	public boolean getSelectButtonReleased(){
		return getRawButtonReleased(Buttons.Select.value);
	}

}