package com.github.mittyrobotics.core.math.geometry

import com.github.mittyrobotics.core.math.units.IN_TO_M
import com.github.mittyrobotics.core.math.units.M_TO_IN
import kotlin.math.pow
import kotlin.math.sqrt

data class Point (var x: Double = 0.0, var y: Double = 0.0) {

    /**
     * Calculates the distance from this [Point] to `other`.
     *
     * @param other the [Point] to find the distance to
     * @return the distance from this [Point] to `other`.
     */
    fun distance(other: Point): Double {
        return sqrt((other.x - x).pow(2.0) + (other.y - y).pow(2.0))
    }

    /**
     * Calculates the magnitude of this [Point] represented as a vector.
     *
     * In other words, this is the distance from this [Point] to (0,0).
     *
     * @return the magnitude of this [Point] represented as a vector.
     */
    fun magnitude(): Double {
        return sqrt(x.pow(2.0) + y.pow(2.0))
    }

    /**
     * Converts this [Point] from inches to meters.
     *
     * @return this [Point] from inches to meters.
     */
    fun inToM(): Point {
        return this * IN_TO_M
    }

    /**
     * Converts this [Point] from meters to inches.
     *
     * @return this [Point] from meters to inches.
     */
    fun mToIn(): Point {
        return this * M_TO_IN
    }

    /**
     * Adds `other` to this [Point].
     *
     * @param other the other [Point] to add to this
     * @return a new [Point] with this and `other` added together
     */
    operator fun plus(other: Point): Point {
        return Point(x + other.x, y + other.y)
    }

    /**
     * Subtracts `other` from this [Point].
     *
     * @param other the other [Point] to subtract this by
     * @return a new [Point] with this subtracted by `other`
     */
    operator fun minus(other: Point): Point {
        return Point(x - other.x, y - other.y)
    }

    /**
     * Multiplies the x and y values of this [Point] by the `scalar`.
     *
     * @param scalar The amount to multiply by
     * @return a new [Point] multiplied by `scalar`.
     */
    operator fun times(scalar: Double): Point {
        return Point(x * scalar, y * scalar)
    }

    /**
     * Divides the x and y values of this [Point] by the `scalar`.
     *
     * @param scalar The amount to divide by
     * @return a new [Point] divided by `scalar`.
     */
    operator fun div(scalar: Double): Point {
        return Point(x / scalar, y / scalar)
    }

    /**
     * Adds `other` to this [Point].
     *
     * @param other the other [Point] to add to this
     */
    operator fun plusAssign(other: Point) {
        x += other.x
        y += other.y
    }

    /**
     * Subtracts `other` from this [Point].
     *
     * @param other the other [Point] to subtract this by
     */
    operator fun minusAssign(other: Point) {
        x -= other.x
        y -= other.y
    }

    /**
     * Multiplies the x and y values of this [Point] by the `scalar`.
     *
     * @param scalar The amount to multiply by
     */
    operator fun timesAssign(scalar: Double) {
        x *= scalar
        y *= scalar
    }

    /**
     * Divides the x and y values of this [Point] by the `scalar`.
     *
     * @param scalar The amount to divide by
     */
    operator fun divAssign(scalar: Double) {
        x /= scalar
        y /= scalar
    }
}