package com.github.mittyrobotics.vision;

import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public class Target {
	private Transform transform;
	private Rotation relativePitch;
	private Rotation relativeYaw;
	
	public Target(Transform transform, Rotation relativePitch, Rotation relativeYaw) {
		this.transform = transform;
		this.relativePitch = relativePitch;
		this.relativeYaw = relativeYaw;
	}
	
	public Transform getTransform() {
		return transform;
	}
	
	public void setTransform(Transform transform) {
		this.transform = transform;
	}
	
	public Rotation getRelativePitch() {
		return relativePitch;
	}
	
	public void setRelativePitch(Rotation relativePitch) {
		this.relativePitch = relativePitch;
	}
	
	public Rotation getRelativeYaw() {
		return relativeYaw;
	}
	
	public void setRelativeYaw(Rotation relativeYaw) {
		this.relativeYaw = relativeYaw;
	}
}
