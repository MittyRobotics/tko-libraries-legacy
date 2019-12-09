package com.github.mittyrobotics.libs.util.path;

import com.github.mittyrobotics.libs.util.geometry.Position;
import com.github.mittyrobotics.libs.util.geometry.Rotation;
import com.github.mittyrobotics.libs.util.geometry.Transform;

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
