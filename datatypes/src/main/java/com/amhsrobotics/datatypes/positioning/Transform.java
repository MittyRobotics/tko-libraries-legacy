package com.amhsrobotics.datatypes.positioning;

/**
 * Represents a 2d {@link Position} and {@link Rotation}
 *
 * Inspired by team 254's Pose2d System: https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/geometry/Pose2d.java
 */
public class Transform {
	private Position position;
	private Rotation rotation;
	
	public Transform(){
		this(new Position(), new Rotation());
	}
	
	public Transform(Position position){
		this(position, new Rotation());
	}
	
	public Transform(Rotation rotation){
		this(new Position(), rotation);
	}
	
	public Transform(double x, double y){
		this(new Position(x,y), new Rotation());
	}
	
	public Transform(double x, double y, double heading){
		this(new Position(x,y), new Rotation(heading));
	}
	
	public Transform(double x, double y, Rotation rotation){
		this(new Position(x,y),rotation);
	}
	
	public Transform(Position position, double heading){
		this(position, new Rotation(heading));
	}
	
	public Transform(Position position, Rotation rotation){
		this.position = position;
		this.rotation = rotation;
	}
	
	/**
	 * Finds this {@link Transform} relative to {@link Transform} <code>other</code>.
	 *
	 * In other words, it finds where this {@link Transform} is if <code>other</code> becomes the new origin
	 * <code(x = 0, y = 0, heading = 0)</code>.
	 *
	 * @param other the {@link Transform} to act as the new origin
	 * @return a new {@link Transform} containing the point relative to <code>other</code>
	 */
	public Transform relativeTo(Transform other){
		return new Transform();
	}
	
	/**
	 * Rotates this {@link Transform} around the given {@link Position} <code>other</code>.
	 *
	 * @param other the {@link Position} to rotate around
	 * @param rotation the {@link Rotation} amount to rotate this Transform around
	 * @return the rotated {@link Transform}
	 */
	public Transform rotateAround(Position other, Rotation rotation){
		return new Transform();
	}
	
	/**
	 * Multiplies all values of this {@link Transform} by the <code>scalar</code>.
	 *
	 * @param scalar The amount to multiply by
	 * @return a new {@link Transform} multiplied by <code>scalar</code>.
	 */
	public Transform multiply(double scalar){
		return new Transform();
	}
	
	/**
	 * Divides all values of this {@link Transform} by the <code>scalar</code>.
	 *
	 * @param scalar The amount to divide by
	 * @return a new {@link Transform} divided by <code>scalar</code>.
	 */
	public Transform divide(double scalar){
		return new Transform();
	}
	
	/**
	 * Adds <code>other</code> to this {@link Transform}.
	 *
	 * @param other the {@link Transform} to add to this {@link Transform}
	 * @return a new {@link Transform} with this {@link Transform} and <code>other</code> added together.
	 */
	public Transform add(Transform other){
		return new Transform();
	}
	
	/**
	 * Subtracts <code>other</code> to this {@link Transform}.
	 *
	 * @param other the {@link Transform} to subtract to this {@link Transform}
	 * @return a new {@link Transform} with this {@link Transform} and <code>other</code> subtracted together.
	 */
	public Transform subtract(Transform other){
		return new Transform();
	}
	
	/**
	 * Transforms this {@link Transform} by <code>other</code>.
	 *
	 * @param other the {@link Transform} to transform this by
	 * @return a new {@link Transform} transformed by <code>other</code>
	 */
	public Transform transformBy(Transform other){
		return new Transform();
	}
	
	public Position getPosition() {
		return position;
	}
	
	public void setPosition(Position position) {
		this.position = position;
	}
	
	public Rotation getRotation() {
		return rotation;
	}
	
	public void setRotation(Rotation rotation) {
		this.rotation = rotation;
	}
}
