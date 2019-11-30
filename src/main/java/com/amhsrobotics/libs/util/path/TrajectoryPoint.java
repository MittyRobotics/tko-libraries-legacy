package com.amhsrobotics.libs.util.path;

import com.amhsrobotics.libs.util.geometry.Position;
import com.amhsrobotics.libs.util.geometry.Rotation;
import com.amhsrobotics.libs.util.geometry.Transform;

public class TrajectoryPoint extends Transform {
	
	private double distanceAlongPath;
	private double time;
	
	
	public TrajectoryPoint(Position position, Rotation rotation) {
		super(position,rotation);
	}
	
	public double getDistanceAlongPath() {
		return distanceAlongPath;
	}
	
	public void setDistanceAlongPath(double distanceAlongPath) {
		this.distanceAlongPath = distanceAlongPath;
	}
	
	public double getTime() {
		return time;
	}
	
	public void setTime(double time) {
		this.time = time;
	}
	
}
