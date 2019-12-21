package com.github.mittyrobotics.path.following.util;

import com.github.mittyrobotics.motionprofile.PathVelocityController;
import com.github.mittyrobotics.path.generation.paths.Path;

public abstract class PathFollowerProperties {
	public final Path path;
	public final PathVelocityController velocityController;
	public final boolean reversed;
	public final boolean adaptivePath;
	public final double robotToPathAdaptiveDistance;
	
	public PathFollowerProperties(Path path,
	                              PathVelocityController velocityController,
	                              boolean reversed,
	                              boolean adaptivePath,
	                              double robotToPathAdaptiveDistance
	){
		this.path = path;
		this.velocityController = velocityController;
		this.reversed = reversed;
		this.adaptivePath = adaptivePath;
		this.robotToPathAdaptiveDistance = robotToPathAdaptiveDistance;
	}
	public static class PurePursuitProperties extends PathFollowerProperties{
		public final double lookaheadDistance;
		public final double curvatureSlowdownGain;
		public final double minSlowdownVelocity;
		public final boolean adaptiveLookahead;
		
		public PurePursuitProperties(Path path,
		                             PathVelocityController velocityController,
		                             boolean reversed,
		                             double lookaheadDistance,
		                             double curvatureSlowdownGain,
		                             double minSlowdownVelocity,
		                             boolean adaptiveLookahead,
		                             boolean adaptivePath,
		                             double robotToPathAdaptiveDistance
		){
			super(path,velocityController,reversed,adaptivePath,robotToPathAdaptiveDistance);
			this.lookaheadDistance = lookaheadDistance;
			this.curvatureSlowdownGain = curvatureSlowdownGain;
			this.minSlowdownVelocity = minSlowdownVelocity;
			this.adaptiveLookahead = adaptiveLookahead;
		}
	}
	public static class RamseteProperties extends PathFollowerProperties{
		public final double aggressiveGain;
		public final double dampingGain;
		
		public RamseteProperties(Path path,
		                         PathVelocityController velocityController,
		                         boolean reversed,
		                         double aggressiveGain,
		                         double dampingGain,
		                         boolean adaptivePath,
		                         double robotToPathAdaptiveDistance
		){
			super(path,velocityController,reversed,adaptivePath,robotToPathAdaptiveDistance);
			this.aggressiveGain = aggressiveGain;
			this.dampingGain = dampingGain;
		}
	}
}
