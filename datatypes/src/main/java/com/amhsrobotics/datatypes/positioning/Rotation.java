package com.amhsrobotics.datatypes.positioning;

/**
 * Represents a 2d rotation (heading) value in degrees or radians and contains the tangent, cosine, or sine of that value.
 * <p>
 * For performance, the trig functions are only calculated on demand, and are only calculated once in each {@link Rotation}.
 * Inspired by team 254's geometry system: https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/geometry/
 */
public class Rotation {
	private double heading;
	private double radians = Double.NaN;
	private double tan = Double.NaN;
	private double cos = Double.NaN;
	private double sin = Double.NaN;
	
	public Rotation() {
		this(0);
	}
	
	public Rotation(double heading) {
		this.heading = heading;
	}
	
	/**
	 * Returns the heading (angle) value in degrees
	 *
	 * @return the heading value in degrees
	 */
	public double getHeading() {
		return heading;
	}
	
	
	/**
	 * Returns the heading (angle) value in radians
	 *
	 * @return the heading value in radians
	 */
	public double getRadians() {
		if (Double.isNaN(radians)) {
			radians = Math.toRadians(getHeading());
		}
		return radians;
	}
	
	/**
	 * Returns the tangent of the radians
	 * <p>
	 * For performance, the tangent is only calculated once in the {@link Rotation}. This function either returns the
	 * already calculated value or calculates it if needed.
	 *
	 * @return the tangent of the radians
	 */
	public double tan() {
		if (Double.isNaN(tan)) {
			tan = Math.tan(getRadians());
		}
		return tan;
	}
	
	/**
	 * Returns the sine of the radians
	 * <p>
	 * For performance, the sine is only calculated once in the {@link Rotation}. This function either returns the
	 * already calculated value or calculates it if needed.
	 *
	 * @return the sine of the radians
	 */
	public double sin() {
		if (Double.isNaN(sin)) {
			sin = Math.sin(getRadians());
		}
		return sin;
	}
	
	/**
	 * Returns the cosine of the radians
	 * <p>
	 * For performance, the cosine is only calculated once in the {@link Rotation}. This function either returns the
	 * already calculated value or calculates it if needed.
	 *
	 * @return the cosine of the radians
	 */
	public double cos() {
		if (Double.isNaN(cos)) {
			cos = Math.cos(getRadians());
		}
		return cos;
	}
	
	/**
	 * Returns the sinc of the radians
	 * <p>
	 * Sinc is sine of radians over radians
	 *
	 * @return the sinc of the radians
	 */
	public double sinc() {
		return sin() / getRadians();
	}
	
	/**
	 * Returns the inverse of the heading value
	 *
	 * @return the inverse of the heading value
	 */
	public double inverseHeading() {
		return -getHeading();
	}
	
	/**
	 * Returns the inverse of the radian value
	 *
	 * @return the inverse of the radian value
	 */
	public double inverseRadians() {
		return -getRadians();
	}
	
	/**
	 * Returns the inverse of the {@link Rotation}
	 *
	 * @return the inverse {@link Rotation} of this
	 */
	public Rotation inverse() {
		return new Rotation(inverseHeading());
	}
	
	/**
	 * Adds <code>other</code> to this {@link Rotation}.
	 *
	 * @param other the other {@link Rotation} to add to this
	 * @return a new {@link Rotation} with this and <code>other</code> added together
	 */
	public Rotation add(Rotation other) {
		return rotateBy(other);
	}
	
	/**
	 * Subtracts <code>other</code> from this {@link Rotation}.
	 *
	 * @param other the other {@link Rotation} to subtract this by
	 * @return a new {@link Rotation} with this subtracted by <code>other</code>
	 */
	public Rotation subtract(Rotation other) {
		return rotateBy(other.inverseHeading());
	}
	
	/**
	 * Multiplies this {@link Rotation} by the <code>scalar</code>.
	 *
	 * @param scalar The amount to multiply by
	 * @return a new {@link Rotation} multiplied by <code>scalar</code>.
	 */
	public Rotation multiply(double scalar) {
		return new Rotation(heading * scalar);
	}
	
	/**
	 * Divides this {@link Rotation} by the <code>scalar</code>.
	 *
	 * @param scalar The amount to divide by
	 * @return a new {@link Rotation} divided by <code>scalar</code>.
	 */
	public Rotation divide(double scalar) {
		return new Rotation(heading / scalar);
	}
	
	public Rotation rotateBy(double otherAngle) {
		return rotateBy(new Rotation(otherAngle));
	}
	
	/**
	 * Rotates this {@link Rotation} by <code>other</code>
	 *
	 * @param other the {@link Rotation} to rotate this {@link Rotation} by
	 * @return a new {@link Rotation} containing the angle of this {@link Rotation} rotated by <code>other</code>
	 */
	public Rotation rotateBy(Rotation other) {
		double cos = cos() * other.cos() - sin() * other.sin();
		double sin = cos() * other.sin() + sin() * other.cos();
		return new Rotation(Math.toDegrees(Math.atan2(sin, cos)));
	}
}
