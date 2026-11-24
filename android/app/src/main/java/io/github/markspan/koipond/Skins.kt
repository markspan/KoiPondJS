package io.github.markspan.koipond

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sqrt
import kotlin.random.Random

private const val WHITE = 0xF4F2EC
private const val BLACK = 0x161418
private const val RED = 0xCC2620

/**
 * A kind of colour patch painted on the body. [radius] is a fraction of the
 * skin height, [xRange] a fraction of the skin width (0 is the nose). Each
 * patch is a cluster of overlapping discs: irregular koi markings.
 */
data class PatchSpec(
    val color: Int,
    val count: IntRange,
    val radius: ClosedFloatingPointRange<Float>,
    val xRange: ClosedFloatingPointRange<Float> = 0.08f..0.66f,
    val blobs: IntRange = 3..6,
)

/** A marking on the head: a round spot (Tancho) or a cap (Soragoi). */
data class HeadSpec(val color: Int, val radius: Float, val x: Float = 0.13f)

/** Colouring of one koi variety. Colours are 0xRRGGBB. */
data class Variety(
    val name: String,
    val base: Int,
    val fin: Int,
    val patches: List<PatchSpec> = emptyList(),
    val head: HeadSpec? = null,
    val metallic: Boolean = false,
    /** Strong reticulated scale net (Asagi, Soragoi). */
    val netted: Boolean = false,
)

val VARIETIES = listOf(
    Variety("Kohaku", WHITE, 0xFAECE8, listOf(PatchSpec(RED, 2..4, 0.12f..0.2f))),
    Variety(
        "Taisho Sanke", WHITE, 0xFAEEEA,
        listOf(
            PatchSpec(RED, 2..3, 0.11f..0.18f),
            PatchSpec(BLACK, 3..6, 0.04f..0.08f, 0.18f..0.72f, 2..4),
        ),
    ),
    Variety(
        "Showa", BLACK, 0x3C383C,
        listOf(PatchSpec(RED, 2..4, 0.13f..0.2f), PatchSpec(WHITE, 1..3, 0.08f..0.14f)),
    ),
    Variety("Tancho", WHITE, 0xF8F4F0, head = HeadSpec(RED, 0.13f)),
    Variety("Orenji Ogon", 0xF48A24, 0xFAAA5A, metallic = true),
    Variety("Ki Bekko", 0xFAC41C, 0xFCD646, listOf(PatchSpec(BLACK, 3..6, 0.04f..0.08f, 0.12f..0.7f, 2..4))),
    Variety("Aka Bekko", 0xBC2222, 0xECAAAA, listOf(PatchSpec(BLACK, 3..6, 0.05f..0.1f, 0.12f..0.7f, 2..5))),
    Variety("Asagi", 0x6080BA, 0xBAD6F0, listOf(PatchSpec(RED, 2..3, 0.1f..0.17f)), netted = true),
    Variety("Hariwake", 0xE6E6E2, 0xF6DEC4, listOf(PatchSpec(0xF08828, 2..4, 0.11f..0.19f)), metallic = true),
    Variety("Soragoi", 0x545E82, 0xB0BEDC, head = HeadSpec(0xF0CEB6, 0.3f, x = 0.05f), netted = true),
)

private fun argb(alpha: Int, rgb: Int) = (alpha shl 24) or (rgb and 0xFFFFFF)

private fun mixRgb(a: Int, b: Int, t: Float): Int {
    fun ch(shift: Int) = lerp(((a shr shift) and 0xFF).toFloat(), ((b shr shift) and 0xFF).toFloat(), t).roundToInt()
    return (ch(16) shl 16) or (ch(8) shl 8) or ch(0)
}

/**
 * Paints a top-view koi texture: head on the left, tail on the right, the
 * pectoral fins reaching the top and bottom edges. The layout matches the
 * skin images of the original sketch, so a fish bends it along its spine.
 */
class SkinPainter(private val w: Int, private val h: Int) {
    private val cy = h / 2f

    private fun halfWidth(x: Float): Float {
        val x0 = NOSE_X * w
        val x1 = TAIL_ROOT_X * w
        val t = (x - x0) / (x1 - x0)
        if (t <= 0f || t > 1f) return 0f
        val widest = BODY_HALF_WIDTH * h
        if (t < 0.25f) return widest * sqrt(max(0f, 1f - ((0.25f - t) / 0.25f).pow(2))) // Rounded head.
        val narrow = PEDUNCLE_HALF_WIDTH * h
        // In float, cos(pi / 2) is -4e-8, and a negative number to the power
        // 1.2 is NaN; one NaN point makes Android discard the whole outline.
        val taper = cos((t - 0.25f) / 0.75f * PI.toFloat() / 2f).coerceAtLeast(0f).pow(1.2f)
        return narrow + (widest - narrow) * taper
    }

    /** Closed body outline; [yScale] < 1 gives a slimmer inner copy. */
    private fun bodyPath(yScale: Float = 1f): Path {
        val x0 = NOSE_X * w
        val x1 = TAIL_ROOT_X * w
        val steps = 40
        val path = Path()
        for (i in 0..steps) {
            val x = x0 + (x1 - x0) * i / steps
            val y = cy - halfWidth(x) * yScale
            if (i == 0) path.moveTo(x, y) else path.lineTo(x, y)
        }
        for (i in steps downTo 0) {
            val x = x0 + (x1 - x0) * i / steps
            path.lineTo(x, cy + halfWidth(x) * yScale)
        }
        path.close()
        return path
    }

    fun paint(variety: Variety, rng: Random): Bitmap {
        val bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        paintFins(canvas, variety.fin)
        canvas.save()
        canvas.clipPath(bodyPath()) // Everything until restore stays on the body.
        paintBody(canvas, variety, rng)
        canvas.restore()
        paintDetails(canvas)
        return bitmap
    }

    private fun fill(color: Int) = Paint(Paint.ANTI_ALIAS_FLAG).apply { this.color = color }

    private fun paintBody(canvas: Canvas, variety: Variety, rng: Random) {
        canvas.drawColor(argb(255, variety.base))
        // Soft shading: darker flanks, lighter along the spine.
        canvas.drawPath(bodyPath(), fill(argb(60, 0)))
        listOf(0.9f, 0.75f, 0.6f, 0.45f, 0.3f).forEachIndexed { i, yScale ->
            canvas.drawPath(bodyPath(yScale), fill(argb(50 - i * 6, 0xFFFFFF)))
        }
        variety.patches.forEach { paintPatches(canvas, it, rng) }
        variety.head?.let { paintHead(canvas, it) }
        paintScales(canvas, variety)
        if (variety.metallic) paintSheen(canvas)
    }

    /** Discs drawn as an opaque core inside a faint halo, for soft edges. */
    private fun paintSoftDiscs(canvas: Canvas, color: Int, discs: List<Triple<Float, Float, Float>>) {
        canvas.saveLayerAlpha(null, 110) // Halos as one layer, so overlaps do not darken.
        val solid = fill(argb(255, color))
        discs.forEach { (x, y, r) -> canvas.drawCircle(x, y, r * 1.2f, solid) }
        canvas.restore()
        discs.forEach { (x, y, r) -> canvas.drawCircle(x, y, r, solid) }
    }

    private fun paintPatches(canvas: Canvas, spec: PatchSpec, rng: Random) {
        val discs = mutableListOf<Triple<Float, Float, Float>>()
        repeat(spec.count.random(rng)) {
            val seedX = rng.nextFloat(spec.xRange) * w
            val spread = halfWidth(seedX) * 0.7f
            val seedY = cy + rng.nextFloat(-spread..spread)
            repeat(spec.blobs.random(rng)) {
                val r = rng.nextFloat(spec.radius) * h
                discs += Triple(
                    seedX + rng.nextFloat(-r..r) * 1.4f,
                    seedY + rng.nextFloat(-r..r) * 0.8f,
                    r,
                )
            }
        }
        paintSoftDiscs(canvas, spec.color, discs)
    }

    private fun paintHead(canvas: Canvas, head: HeadSpec) {
        val r = head.radius * h
        canvas.saveLayerAlpha(null, 120)
        canvas.drawCircle(head.x * w, cy, r * 1.12f, fill(argb(255, head.color)))
        canvas.restore()
        canvas.drawCircle(head.x * w, cy, r, fill(argb(255, head.color)))
    }

    /** Rows of arcs bulging towards the tail, like overlapping scales. */
    private fun paintScales(canvas: Canvas, variety: Variety) {
        val darkBase = Color.red(variety.base) + Color.green(variety.base) + Color.blue(variety.base) < 300
        val color = when {
            variety.netted -> argb(90, 0xEBF0FA)
            darkBase -> argb(40, 0xFFFFFF)
            else -> argb(34, 0)
        }
        val stroke = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            this.color = color
            style = Paint.Style.STROKE
            strokeWidth = max(1f, h / 80f)
        }
        val r = max(2f, 0.055f * h)
        val startX = (NOSE_X + 0.2f) * w // No scales on the head.
        var y = cy - BODY_HALF_WIDTH * h
        var row = 0
        val rect = RectF()
        while (y < cy + BODY_HALF_WIDTH * h) {
            var x = startX + if (row % 2 == 1) r * 0.8f else 0f
            while (x < TAIL_ROOT_X * w) {
                rect.set(x - r, y - r, x + r, y + r)
                canvas.drawArc(rect, -90f, 180f, false, stroke)
                x += r * 1.6f
            }
            y += r * 0.95f
            row++
        }
    }

    /** Metallic varieties get a bright band along the back. */
    private fun paintSheen(canvas: Canvas) {
        val paint = fill(argb(34, 0xFFFFF0))
        for (i in 0 until 4) {
            val halfLen = 0.31f * w * (1 - i * 0.12f)
            val halfHeight = (0.14f - i * 0.03f) * h
            canvas.drawOval(0.38f * w - halfLen, cy - halfHeight, 0.38f * w + halfLen, cy + halfHeight, paint)
        }
    }

    private fun quad(p0: Pair<Float, Float>, c: Pair<Float, Float>, p1: Pair<Float, Float>, t: Float): Pair<Float, Float> {
        val u = 1 - t
        return Pair(
            u * u * p0.first + 2 * u * t * c.first + t * t * p1.first,
            u * u * p0.second + 2 * u * t * c.second + t * t * p1.second,
        )
    }

    private fun paintFins(canvas: Canvas, color: Int) {
        val finPaint = fill(argb(200, color))
        val rayPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            this.color = argb(110, mixRgb(color, 0x5A5A5A, 0.5f))
            style = Paint.Style.STROKE
            strokeWidth = max(1f, h / 100f)
        }

        // Tail fin, with a notch in the middle.
        val root = TAIL_ROOT_X * w - 0.02f * w
        val tail = Path().apply {
            moveTo(root, cy - 0.05f * h)
            lineTo(0.985f * w, cy - 0.23f * h)
            lineTo(0.93f * w, cy)
            lineTo(0.985f * w, cy + 0.23f * h)
            lineTo(root, cy + 0.05f * h)
            close()
        }
        canvas.drawPath(tail, finPaint)
        for (k in -3..3) {
            val tipX = if (k == 0) 0.94f * w else 0.97f * w - kotlin.math.abs(k) * 0.004f * w
            canvas.drawLine(root, cy, tipX, cy + k * 0.065f * h, rayPaint)
        }

        // Pectoral fins, top and bottom, swept back like leaves.
        for (side in intArrayOf(-1, 1)) {
            val front = Pair(0.19f * w, cy + side * halfWidth(0.19f * w) * 0.9f)
            val back = Pair(0.29f * w, cy + side * halfWidth(0.29f * w) * 0.9f)
            val tip = Pair(0.265f * w, cy + side * 0.48f * h)
            val c1 = Pair(0.19f * w, cy + side * 0.42f * h)
            val c2 = Pair(0.33f * w, cy + side * 0.44f * h)
            val fin = Path().apply {
                moveTo(front.first, front.second)
                quadTo(c1.first, c1.second, tip.first, tip.second)
                quadTo(c2.first, c2.second, back.first, back.second)
                close()
            }
            canvas.drawPath(fin, finPaint)
            for (k in 0 until 4) {
                val (x, y) = if (k < 2) quad(front, c1, tip, 0.7f + 0.3f * k) else quad(tip, c2, back, 0.15f * (k - 1))
                canvas.drawLine(front.first, front.second, x, y, rayPaint)
            }
        }
    }

    /** Outline and eyes on top of everything else. */
    private fun paintDetails(canvas: Canvas) {
        val outline = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = argb(170, 0x141418)
            style = Paint.Style.STROKE
            strokeWidth = max(1f, h / 90f)
        }
        canvas.drawPath(bodyPath(), outline)
        val eye = fill(argb(255, 0x0C0C10))
        val eyeR = max(1f, 0.032f * h)
        for (side in intArrayOf(-1, 1)) canvas.drawCircle(0.075f * w, cy + side * 0.15f * h, eyeR, eye)
    }

    companion object {
        private const val NOSE_X = 0.02f // Body extent, as fractions of the width.
        private const val TAIL_ROOT_X = 0.80f
        private const val BODY_HALF_WIDTH = 0.29f // Fractions of the height.
        private const val PEDUNCLE_HALF_WIDTH = 0.05f

        /** A fresh skin of a random variety for a fish at [scale]. */
        fun randomSkin(config: PondConfig, scale: Float, rng: Random): Pair<Bitmap, String> {
            val size = scale * rng.nextFloat(config.minSkinScale..config.maxSkinScale)
            val painter = SkinPainter(
                max(16, (config.skinWidth * size).roundToInt()),
                max(8, (config.skinHeight * size).roundToInt()),
            )
            val variety = VARIETIES.random(rng)
            return painter.paint(variety, rng) to variety.name
        }
    }
}

/** A uniformly random float in [range]. */
fun Random.nextFloat(range: ClosedFloatingPointRange<Float>) =
    range.start + nextFloat() * (range.endInclusive - range.start)
