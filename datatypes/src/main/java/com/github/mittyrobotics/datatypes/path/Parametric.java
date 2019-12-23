package com.github.mittyrobotics.datatypes.path;

import com.github.mittyrobotics.datatypes.positioning.Position;
import com.github.mittyrobotics.datatypes.positioning.Rotation;
import com.github.mittyrobotics.datatypes.positioning.Transform;

public interface Parametric {
	/**
	 * Returns the {@link Position} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
	 *
	 * @param t the parameter
	 * @return the {@link Position} at the parameter <code>t</code>.
	 */
	public abstract Position getPosition(double t);
	
	/**
	 * Returns the {@link Transform} along the {@link Parametric} at <code>t</code> where <code>0 <= t <= 1</code>.
	 * <p>
	 * The {@link Transform} contains the {@link Position} and {@link Rotation}, with the {@link Rotation} being the
	 * tangent angle at the {@link Position}.
	 *
	 * @param t the parameter
	 * @return the {@link Transform} at the parameter <code>t</code>.
	 */
	public abstract Transform getTransform(double t);
	
	/**
	 * Returns the curvature at point <code>t</code> on the {@link Parametric}.
	 *
	 * @param t the parameter
	 * @return the curvature at the parameter <code>t</code>.
	 */
	public abstract double getCurvature(double t);
}
