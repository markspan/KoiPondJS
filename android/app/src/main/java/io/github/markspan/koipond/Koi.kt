package io.github.markspan.koipond

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LightingColorFilter
import android.graphics.Paint
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * A chain of nodes that swims like a fish body.
 *
 * Logic from levitated.com via the original sketch: the neck node swings
 * with a sine "muscle" and every following node is dragged along at a fixed
 * distance, so the wave travels down to the tail. Node positions are offsets
 * from the fish's location; node 0 is the head.
 */
class Flagellum(length: Float, private val config: PondConfig, rng: Random) {
    val segments = config.bodySegments
    private val spacing = length / segments
    private var musclePhase = rng.nextFloat() * TAU
    var muscleRate = 0f
    var heading = rng.nextFloat() * TAU
    val nodes: Array<Vec2> = Array(segments + 1) { n -> -Vec2.polar(heading, spacing * n) }

    /** Swing the neck by [dtFrames] (frames at 60 fps) and drag the spine. */
    fun move(dtFrames: Float) {
        val head = Vec2.polar(heading)
        nodes[0] = head
        musclePhase += muscleRate * dtFrames
        val neck = heading + Math.toRadians(config.muscleRangeDeg * sin(musclePhase).toDouble()).toFloat()
        nodes[1] = head - Vec2.polar(neck, spacing)
        for (n in 2..segments) {
            var pull = nodes[n] - nodes[n - 2]
            if (pull.lengthSquared() == 0f) pull = -head
            nodes[n] = nodes[n - 1] + pull.withLength(spacing)
        }
    }
}

/** A wandering koi: calm by default, stirred by touches, bolting when near one. */
class Koi(
    private val skin: Bitmap,
    val variety: String,
    position: Vec2,
    /** Depth below the surface; sets how much the water refracts it. */
    private val depthM: Float,
    brightness: Int,
    private val scale: Float,
    private val config: PondConfig,
    private val rng: Random,
) {
    private val body = Flagellum(skin.width.toFloat(), config, rng)
    var position = position
        private set
    private var velocity = Vec2.ZERO
    private var acceleration = Vec2.ZERO
    private val pace = rng.nextFloat() // Where this fish sits within the speed ranges.
    private var stir = 0f
    private var maxForce = config.calmForce * scale
    private var maxSpeed = cruiseSpeed()
    private val rushSpeed = rng.nextFloat(config.rushSpeedMin..config.rushSpeedMax) * scale
    private var wanderTheta = rng.nextFloat() * TAU
    private var opacity = 0f

    // Rush after a scare: steps down in fixed time steps, null when over.
    private var rushStep: Int? = null
    private var rushTimerMs = 0f

    // Drawing: the skin is bent along the spine as a 1-row bitmap mesh, like
    // the original sketch's textured QUAD_STRIP. Depth darkens it.
    private val paint = Paint(Paint.FILTER_BITMAP_FLAG or Paint.ANTI_ALIAS_FLAG).apply {
        colorFilter = LightingColorFilter(Color.rgb(brightness, brightness, brightness), 0)
    }
    private val vertices = FloatArray(4 * (body.segments + 1))
    private val shift = FloatArray(2)

    private fun cruiseSpeed(): Float {
        val calm = lerp(config.calmSpeedMin, config.calmSpeedMax, pace)
        val stirred = lerp(config.stirredSpeedMin, config.stirredSpeedMax, pace)
        return lerp(calm, stirred, stir) * scale
    }

    // -- steering -------------------------------------------------------------

    /** Steering force towards [target] at full speed (desired minus velocity). */
    private fun steer(target: Vec2): Vec2 {
        val desired = target - position
        if (desired.lengthSquared() == 0f) return Vec2.ZERO
        return (desired.withLength(maxSpeed) - velocity).limited(maxForce)
    }

    /** Seek a point that drifts around a circle ahead of the fish. */
    fun wander(dtFrames: Float) {
        val jitter = lerp(config.calmWanderJitter, config.stirredWanderJitter, stir)
        // A random walk: its spread grows with the square root of time.
        wanderTheta += (rng.nextFloat() * 2f - 1f) * jitter * sqrt(dtFrames)
        val circle = position + Vec2.polar(body.heading, config.wanderDistance * scale)
        val target = circle + Vec2.polar(wanderTheta, config.wanderRadius * scale)
        acceleration += steer(target)
    }

    /** Steer away from [threat] and start (or restart) a rush. */
    fun flee(threat: Vec2) {
        maxForce = config.stirredForce * scale
        acceleration -= steer(threat)
        rushStep = 0
        rushTimerMs = config.rushStepMs // The first rush step happens at once.
    }

    // -- motion ---------------------------------------------------------------

    /** Move by [dtFrames] (frames at 60 fps); [pondStir] is 0 calm to 1 stirred. */
    fun update(dtFrames: Float, pondStir: Float) {
        stir = pondStir
        if (rushStep == null) {
            // Ease towards the speed and agility that suit the stir level.
            val ease = 1f - (1f - config.speedEase).pow(dtFrames)
            maxSpeed += (cruiseSpeed() - maxSpeed) * ease
            maxForce = lerp(config.calmForce, config.stirredForce, stir) * scale
        }
        velocity = (velocity + acceleration * dtFrames).limited(maxSpeed)
        position += velocity * dtFrames
        acceleration = Vec2.ZERO

        if (velocity.lengthSquared() > 1e-6f) body.heading = atan2(velocity.y, velocity.x)
        body.muscleRate = velocity.length() / scale * config.muscleRatePerSpeed
        body.move(dtFrames)
        updateRush(dtFrames * 1000f / 60f)
        opacity = min(255f, opacity + config.fadeInPerFrame * dtFrames)
    }

    /** After a scare, swim fast and slow down step by step; then [update] eases. */
    private fun updateRush(dtMs: Float) {
        val step = rushStep ?: return
        rushTimerMs += dtMs
        if (rushTimerMs < config.rushStepMs) return
        rushTimerMs = 0f
        if (step <= config.rushSteps) {
            maxSpeed = rushSpeed - config.rushSlowdown * scale * step / config.rushSteps
            rushStep = step + 1
        } else {
            rushStep = null
        }
    }

    /** Reappear on the opposite side once fully off screen. */
    fun wrap(width: Int, height: Int) {
        val m = skin.width.toFloat()
        var x = position.x
        var y = position.y
        if (x < -m) x = width.toFloat() else if (x > width + m) x = 0f
        if (y < -m) y = height.toFloat() else if (y > height + m) y = 0f
        position = Vec2(x, y)
    }

    fun isVisible(width: Int, height: Int): Boolean {
        val m = skin.width.toFloat()
        return position.x in -m..width + m && position.y in -m..height + m
    }

    /** Draw the skin bent along the spine, shifted by the water's refraction. */
    fun draw(canvas: Canvas, water: WaterSurface) {
        val nodes = body.nodes
        val columns = nodes.size
        val halfHeight = skin.height / 2f
        for (n in 0 until columns) {
            // Segment direction (head to tail) at this node; its normal spans the skin.
            val from = if (n == 0) nodes[0] else nodes[n - 1]
            val to = if (n == 0) nodes[1] else nodes[n]
            val angle = atan2(to.y - from.y, to.x - from.x)
            val nx = sin(angle) * halfHeight
            val ny = -cos(angle) * halfHeight
            var x = position.x + nodes[n].x
            var y = position.y + nodes[n].y
            if (water.refraction(x, y, depthM, shift)) {
                x += shift[0]
                y += shift[1]
            }
            // Row 0 (texture top) first, then row 1 (texture bottom).
            vertices[2 * n] = x + nx
            vertices[2 * n + 1] = y + ny
            vertices[2 * (columns + n)] = x - nx
            vertices[2 * (columns + n) + 1] = y - ny
        }
        paint.alpha = opacity.toInt()
        canvas.drawBitmapMesh(skin, columns - 1, 1, vertices, 0, null, 0, paint)
    }
}
