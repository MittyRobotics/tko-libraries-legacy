package com.amhsrobotics.libs.math.geometry;

public class Line {
	
	private Transform transform;
	
	public Line(Transform transform){
		
		this.transform = transform;
	}
	
	public Position getP1(){
		return transform.getPosition();
	}
	
	public Position getP2(){
		return new Position(transform.getRotation().cos(), transform.getRotation().sin()).add(transform.getPosition());
	}
	
	public double getSlope(){
		double slope = (getP2().getY()-getP1().getY())/(getP2().getX()- getP1().getX());
		if (Double.isInfinite(slope)) {
			return 999999999;
		}
		return slope;
	}
	
	public double getYIntercept(){
		return getP1().getY() - getSlope() * getP1().getX();
	}
	
	public Transform getTransform() {
		return transform;
	}
	
	public void setTransform(Transform transform) {
		this.transform = transform;
	}
}
