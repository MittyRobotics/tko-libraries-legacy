package com.github.mittyrobotics.datatypes.positioning;

/**
 * Represents a 2d {@link Position} and {@link Rotation}
 * <p>
 * Inspired by team 254's geometry system:
 * https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/geometry/
 */
public class Transform {
	private Position position;
	private Rotation rotation;
	
	public Transform() {
		this(new Position(), new Rotation());
	}
	
	public Transform(Position position) {
		this(position, new Rotation());
	}
	
	public Transform(Rotation rotation) {
		this(new Position(), rotation);
	}
	
	public Transform(double x, double y) {
		this(new Position(x, y), new Rotation());
	}
	
	public Transform(double x, double y, double heading) {
		this(new Position(x, y), new Rotation(heading));
	}
	
	public Transform(double x, double y, Rotation rotation) {
		this(new Position(x, y), rotation);
	}
	
	public Transform(Position position, double heading) {
		this(position, new Rotation(heading));
	}
	
	public Transform(Position position, Rotation rotation) {
		this.position = position;
		this.rotation = rotation;
	}
	
	public Transform(Transform transform) {
		this.position = transform.getPosition();
		this.rotation = transform.getRotation();
	}
	
	/**
	 * Converts this {@link Transform} from inches to meters.
	 *
	 * @return this {@link Transform} from inches to meters.
	 */
	public Transform inToM() {
		return new Transform(position.inToM(), rotation);
	}
	
	/**
	 * Converts this {@link Transform} from meters to inches.
	 *
	 * @return this {@link Transform} from meters to inches.
	 */
	public Transform mToIn() {
		return new Transform(position.mToIn(), rotation);
	}
	
	/**
	 * Rotates the {@link Rotation} of this {@link Transform} by <code>rotation</code>.
	 *
	 * @param other the {@link Rotation} to rotate this {@link Rotation} by.
	 * @return a new {@link Transform} with the {@link Rotation} rotated by <code>rotation</code>.
	 */
	public Transform rotateBy(Rotation other) {
		return new Transform(position, rotation.add(other));
	}
	
	/**
	 * Finds this {@link Transform} relative to {@link Transform} <code>other</code>.
	 * <p>
	 * In other words, it finds where this {@link Transform} is if <code>other</code> becomes the new origin
	 * <code(x = 0, y = 0, heading = 0)</code>.
	 *
	 * @param other the {@link Transform} to act as the new origin
	 * @return a new {@link Transform} containing the point relative to <code>other</code>
	 */
	public Transform relativeTo(Transform other) {
		Position pos = position.subtract(other.getPosition()).rotateBy(other.getRotation().inverse());
		Rotation rot = rotation.subtract(other.getRotation());
		
		return new Transform(pos, rot);
	}
	
	/**
	 * Rotates this {@link Transform} around the given {@link Position} <code>other</code>.
	 *
	 * @param origin   the {@link Position} to rotate around
	 * @param rotation the {@link Rotation} amount to rotate this Transform around
	 * @return the rotated {@link Transform}
	 */
	public Transform rotateAround(Position origin, Rotation rotation) {
		Position pos = position.subtract(origin).rotateBy(rotation).add(origin);
		Rotation rot = rotation.add(rotation);
		
		return new Transform(pos, rot);
	}
	
	/**
	 * Multiplies all values of this {@link Transform} by the <code>scalar</code>.
	 *
	 * @param scalar The amount to multiply by
	 * @return a new {@link Transform} multiplied by <code>scalar</code>.
	 */
	public Transform multiply(double scalar) {
		Position pos = position.multiply(scalar);
		Rotation rot = rotation.multiply(scalar);
		
		return new Transform(pos, rot);
	}
	
	/**
	 * Divides all values of this {@link Transform} by the <code>scalar</code>.
	 *
	 * @param scalar The amount to divide by
	 * @return a new {@link Transform} divided by <code>scalar</code>.
	 */
	public Transform divide(double scalar) {
		Position pos = position.divide(scalar);
		Rotation rot = rotation.divide(scalar);
		
		return new Transform(pos, rot);
	}
	
	/**
	 * Adds <code>other</code> to this {@link Transform}.
	 *
	 * @param other the {@link Transform} to add to this {@link Transform}
	 * @return a new {@link Transform} with this {@link Transform} and <code>other</code> added together.
	 */
	public Transform add(Transform other) {
		Position pos = position.add(other.getPosition());
		Rotation rot = rotation.add(other.getRotation());
		
		return new Transform(pos, rot);
	}
	
	/**
	 * Subtracts <code>other</code> to this {@link Transform}.
	 *
	 * @param other the {@link Transform} to subtract to this {@link Transform}
	 * @return a new {@link Transform} with this {@link Transform} and <code>other</code> subtracted together.
	 */
	public Transform subtract(Transform other) {
		Position pos = position.subtract(other.getPosition());
		Rotation rot = rotation.subtract(other.getRotation());
		
		return new Transform(pos, rot);
	}
	
	/**
	 * Transforms this {@link Transform} by <code>other</code>.
	 *
	 * @param other the {@link Transform} to transform this by
	 * @return a new {@link Transform} transformed by <code>other</code>
	 */
	public Transform transformBy(Transform other) {
		Position pos = position.add(other.getPosition().rotateBy(rotation));
		Rotation rot = rotation.add(other.getRotation());
		
		return new Transform(pos, rot);
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
	
	@Override
	public String toString() {
		return String.format("Transform(%s, %s)", position.toString(), rotation.toString());
	}
}
