package io.github.markspan.koipond

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** An immutable 2-D vector with the few operations steering needs. */
data class Vec2(val x: Float, val y: Float) {
    operator fun plus(o: Vec2) = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2) = Vec2(x - o.x, y - o.y)
    operator fun times(s: Float) = Vec2(x * s, y * s)
    operator fun div(s: Float) = Vec2(x / s, y / s)
    operator fun unaryMinus() = Vec2(-x, -y)

    fun lengthSquared() = x * x + y * y
    fun length() = sqrt(lengthSquared())

    fun distanceSquaredTo(o: Vec2): Float {
        val dx = x - o.x
        val dy = y - o.y
        return dx * dx + dy * dy
    }

    /** This vector scaled to [length]; the zero vector stays zero. */
    fun withLength(length: Float): Vec2 {
        val l = length()
        return if (l == 0f) this else this * (length / l)
    }

    /** This vector clamped to at most [max] long (p5's `limit`). */
    fun limited(max: Float): Vec2 {
        val l2 = lengthSquared()
        return if (l2 > max * max) this * (max / sqrt(l2)) else this
    }

    companion object {
        val ZERO = Vec2(0f, 0f)

        /** Unit vector (times [length]) at [angle] radians, clockwise on screen. */
        fun polar(angle: Float, length: Float = 1f) = Vec2(cos(angle) * length, sin(angle) * length)
    }
}

const val TAU = (2.0 * Math.PI).toFloat()

fun lerp(a: Float, b: Float, t: Float) = a + (b - a) * t
