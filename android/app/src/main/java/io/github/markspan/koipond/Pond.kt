package io.github.markspan.koipond

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.Shader
import kotlin.math.exp
import kotlin.math.min
import kotlin.random.Random

/**
 * The whole pond for one screen size: scenery, water and the school of koi.
 *
 * Not thread-safe: the wallpaper's render thread owns it. Touches arrive as
 * a finger position (or null) passed to [step].
 */
class Pond(private val width: Int, private val height: Int, private val config: PondConfig = PondConfig()) {
    private val rng = Random(System.nanoTime())
    private val scale = min(width, height) / config.referenceShortSide
    private val background = paintPebbleBed()
    private val rimShadow = RimShadow(width, height, (60 * scale).toInt().coerceAtLeast(1))
    private val water = WaterSurface(width, height, config)

    // Every koi is prepared now (painting a skin takes longer than a frame),
    // so births later cost nothing. Earlier-born fish swim deeper.
    private val unborn = ArrayDeque((0 until config.maxKoi).map(::prepareKoi))
    private val school = mutableListOf<Koi>()
    private var sinceBirthMs = 0f

    /** 0 calm to 1 stirred: set by touches, fades exponentially. */
    private var stir = 0f

    init {
        if (config.startWithFullPond) {
            school += unborn
            unborn.clear()
        }
    }

    private fun prepareKoi(birthOrder: Int): Koi {
        val rank = birthOrder / (config.maxKoi - 1).coerceAtLeast(1).toFloat() // 0 deepest, 1 highest.
        val brightness = lerp(config.depthBrightnessDeep.toFloat(), config.depthBrightnessShallow.toFloat(), rank).toInt()
        val depthM = lerp(config.fishDepthDeepM, config.fishDepthShallowM, rank)
        val margin = min(100 * scale, min(width, height) / 4f)
        val position = Vec2(
            rng.nextFloat(margin..width - margin),
            rng.nextFloat(margin..height - margin),
        )
        val (skin, variety) = SkinPainter.randomSkin(config, scale, rng)
        return Koi(skin, variety, position, depthM, brightness, scale, config, rng)
    }

    /** Advance by [dtS] seconds; [finger] is the current touch, or null. */
    fun step(dtS: Float, finger: Vec2?) {
        val dtFrames = dtS * 60f // Behaviour constants are per frame at 60 fps.
        sinceBirthMs += dtS * 1000f
        if (unborn.isNotEmpty() && sinceBirthMs >= config.birthIntervalMs) {
            school += unborn.removeFirst()
            sinceBirthMs = 0f
        }

        stir = if (finger != null) 1f else stir * exp(-dtS / config.stirFadeS)
        val scareRadiusSq = (config.scareRadius * scale).let { it * it }
        for (koi in school) {
            if (finger != null && koi.position.distanceSquaredTo(finger) <= scareRadiusSq) {
                koi.flee(finger)
            } else {
                koi.wander(dtFrames)
            }
            koi.update(dtFrames, stir)
            koi.wrap(width, height)
        }
        water.update(dtS, finger)
    }

    fun draw(canvas: Canvas) {
        canvas.drawBitmap(background, 0f, 0f, null)
        for (koi in school) if (koi.isVisible(width, height)) koi.draw(canvas, water)
        water.draw(canvas)
        rimShadow.draw(canvas)
    }

    /** A dark bed of rounded pebbles, painted once. */
    private fun paintPebbleBed(): Bitmap {
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        canvas.drawColor(Color.rgb(3, 6, 12))
        val stone = Paint(Paint.ANTI_ALIAS_FLAG)
        val count = (width.toLong() * height / (260 * scale * scale)).toInt()
        repeat(count) {
            val rx = rng.nextFloat(6f..16f) * scale
            val ry = rx * rng.nextFloat(0.55f..0.85f)
            val x = rng.nextFloat(-rx..width + rx)
            val y = rng.nextFloat(-ry..height + ry)
            val shade = rng.nextFloat()
            val r = lerp(9f, 26f, shade).toInt()
            val g = lerp(14f, 36f, shade).toInt()
            val b = lerp(26f, 56f, shade).toInt()
            canvas.save()
            canvas.rotate(rng.nextFloat(0f..180f), x, y)
            stone.color = Color.rgb(r, g, b)
            canvas.drawOval(x - rx, y - ry, x + rx, y + ry, stone)
            // A faint highlight towards the top left.
            stone.color = Color.argb(70, lerp(r.toFloat(), 70f, 0.2f).toInt(), lerp(g.toFloat(), 84f, 0.2f).toInt(), lerp(b.toFloat(), 110f, 0.2f).toInt())
            val hx = x - rx * 0.2f
            val hy = y - ry * 0.25f
            canvas.drawOval(hx - rx * 0.55f, hy - ry * 0.55f, hx + rx * 0.55f, hy + ry * 0.55f, stone)
            canvas.restore()
        }
        return bitmap
    }

    /** Dark rim around the pond, drawn above the fish like the original. */
    private class RimShadow(private val width: Int, private val height: Int, private val depth: Int) {
        private fun edge(x0: Float, y0: Float, x1: Float, y1: Float) = Paint().apply {
            // Alpha falls off as 230 t^2 from the edge inwards, as in Python.
            val stops = floatArrayOf(0f, 0.25f, 0.5f, 0.75f, 1f)
            val colors = stops.map { Color.argb((230 * (1 - it) * (1 - it)).toInt(), 0, 0, 0) }.toIntArray()
            shader = LinearGradient(x0, y0, x1, y1, colors, stops, Shader.TileMode.CLAMP)
        }

        private val d = depth.toFloat()
        private val left = edge(0f, 0f, d, 0f)
        private val right = edge(width.toFloat(), 0f, width - d, 0f)
        private val top = edge(0f, 0f, 0f, d)
        private val bottom = edge(0f, height.toFloat(), 0f, height - d)

        fun draw(canvas: Canvas) {
            canvas.drawRect(0f, 0f, d, height.toFloat(), left)
            canvas.drawRect(width - d, 0f, width.toFloat(), height.toFloat(), right)
            canvas.drawRect(0f, 0f, width.toFloat(), d, top)
            canvas.drawRect(0f, height - d, width.toFloat(), height.toFloat(), bottom)
        }
    }
}
