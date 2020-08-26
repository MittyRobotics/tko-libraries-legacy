/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.datatypes.positioning;

/**
 * Represents a 2d rotation (heading) value in degrees or radians and contains the tangent, cosine, or sine of that
 * value.
 * <p>
 * For performance, the trig functions are only calculated on demand, and are only calculated once in each {@link
 * Rotation}. Inspired by team 254's geometry system: https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/geometry/
 */
public class Rotation {
    private double radians;
    private double degrees = Double.NaN;
    private double tan = Double.NaN;
    private double cos = Double.NaN;
    private double sin = Double.NaN;

    public Rotation() {
        this(0);
    }

    public Rotation(double radians) {
        this.radians = radians;
    }

    /**
     * Returns a {@link Rotation} from an input degree angle
     *
     * @param degrees degree angle
     * @return a new {@link Rotation} from the input degree angle
     */
    public static Rotation fromDegrees(double degrees) {
        return new Rotation(Math.toRadians(degrees));
    }

    /**
     * Returns a {@link Rotation} from an input radian angle
     *
     * @param radians radian angle
     * @return a new {@link Rotation} from the input radian angle
     */
    public static Rotation fromRadians(double radians) {
        return new Rotation(radians);
    }

    /**
     * Returns the angle value in radians
     *
     * @return the angle value in radians
     */
    public double getRadians() {
        return radians;
    }

    /**
     * Returns the angle value in degrees
     *
     * @return the angle value in degrees
     */
    public double getDegrees() {
        if (Double.isNaN(degrees)) {
            degrees = Math.toDegrees(getRadians());
        }
        return degrees;
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
        if (getRadians() == 0) {
            return 0;
        }
        return sin() / getRadians();
    }

    /**
     * Returns the inverse of the radians value
     *
     * @return the inverse of the radians value
     */
    public double inverseRadians() {
        return -getRadians();
    }

    /**
     * Returns the inverse of the degree value
     *
     * @return the inverse of the degree value
     */
    public double inverseDegrees() {
        return -getDegrees();
    }

    /**
     * Returns the inverse of the {@link Rotation}
     *
     * @return the inverse {@link Rotation} of this
     */
    public Rotation inverse() {
        return new Rotation(inverseRadians());
    }

    /**
     * Adds <code>other</code> to this {@link Rotation}.
     *
     * @param other the other {@link Rotation} to add to this
     * @return a new {@link Rotation} with this and <code>other</code> added together
     */
    public Rotation add(Rotation other) {
        return new Rotation(getRadians() + other.getRadians());
    }

    /**
     * Subtracts <code>other</code> from this {@link Rotation}.
     *
     * @param other the other {@link Rotation} to subtract this by
     * @return a new {@link Rotation} with this subtracted by <code>other</code>
     */
    public Rotation subtract(Rotation other) {
        return new Rotation(getRadians() - other.getRadians());
    }

    /**
     * Multiplies this {@link Rotation} by the <code>scalar</code>.
     *
     * @param scalar The amount to multiply by
     * @return a new {@link Rotation} multiplied by <code>scalar</code>.
     */
    public Rotation multiply(double scalar) {
        return new Rotation(radians * scalar);
    }

    /**
     * Divides this {@link Rotation} by the <code>scalar</code>.
     *
     * @param scalar The amount to divide by
     * @return a new {@link Rotation} divided by <code>scalar</code>.
     */
    public Rotation divide(double scalar) {
        return new Rotation(radians / scalar);
    }

    /**
     * Rotates this {@link Rotation} by the <code>angle</code> in radians
     *
     * @param radians the angle in radians to rotate this {@link Rotation} by
     * @return a new {@link Rotation} containing the angle of this {@link Rotation} rotated by <code>angle</code>.
     */
    public Rotation rotateByRadians(double radians) {
        return rotateBy(new Rotation(radians));
    }

    /**
     * Rotates this {@link Rotation} by the <code>angle</code> in degrees
     *
     * @param degrees the angle in degrees to rotate this {@link Rotation} by
     * @return a new {@link Rotation} containing the angle of this {@link Rotation} rotated by <code>angle</code>.
     */
    public Rotation rotateByDegrees(double degrees) {
        return rotateBy(Rotation.fromDegrees(degrees));
    }

    public Rotation abs() {
        return Rotation.fromRadians(Math.abs(radians));
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

    /**
     * Maps the degree value of the {@link Rotation} between -180 and 180;
     *
     * @return a new {@link Rotation} with the degree mapped between -180 and 180;
     */
    public Rotation mapDegrees180() {
        double angle = getRadians();
        double sign = Math.signum(angle);
        angle = Math.abs(angle % 360);
        if (angle <= 180 && angle >= 0) {
            return Rotation.fromDegrees((angle * sign));
        } else {
            return Rotation.fromDegrees((sign * ((angle % 360) % 180 - 180)));
        }
    }

    /**
     * Maps the degree value of the {@link Rotation} between 0 and 360;
     *
     * @return a new {@link Rotation} with the degree mapped between 0 and 360;
     */
    public Rotation mapDegrees360() {
        double angle = mapDegrees180().getRadians();
        if (angle < 0) {
            return Rotation.fromDegrees(angle + 360);
        } else {
            return Rotation.fromDegrees(angle);
        }
    }

    @Override
    public String toString() {
        return String.format("Rotation(%s)", radians);
    }
}
