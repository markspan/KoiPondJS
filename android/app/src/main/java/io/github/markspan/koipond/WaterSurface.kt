package io.github.markspan.koipond

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.RectF
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Physically based ripples on the pond, as in the Python version.
 *
 * The height field lives in Fourier space, where linear water waves are
 * independent damped oscillators, so each mode is advanced exactly:
 *
 *     omega0(k)^2 = g k + (sigma / rho) k^3     deep-water dispersion
 *     beta(k)     = film + 2 nu k^2              viscous + film damping
 *
 * Surface tension makes short ripples outrun long ones, so a touch spreads
 * into a train of rings. The finger is a pressure patch on the surface. The
 * surface is drawn through optics: caustics on the floor (-D (1 - 1/n)
 * laplacian h), glints from the slope, and refraction of the fish
 * (-d (1 - 1/n) grad h, Snell's law for small slopes).
 *
 * Nothing is computed while the water is calm, which is the normal state on
 * the lock screen.
 */
class WaterSurface(width: Int, height: Int, private val config: PondConfig) {
    private val portrait = width <= height
    private val nx = if (portrait) config.waterGridShort else config.waterGridLong
    private val ny = if (portrait) config.waterGridLong else config.waterGridShort
    private val size = nx * ny

    private val cellPx = max(
        width / (nx * (1 - 2 * config.waterMargin)),
        height / (ny * (1 - 2 * config.waterMargin)),
    )
    private val pxPerM = min(width, height) / config.pondShortSideM
    private val cellM = cellPx / pxPerM
    private val visibleX = min(nx, ceil(width / cellPx).toInt())
    private val visibleY = min(ny, ceil(height / cellPx).toInt())
    private val padX = (nx - visibleX) / 2
    private val padY = (ny - visibleY) / 2

    private val kx = FloatArray(nx) { (2 * PI * frequency(it, nx) / (nx * cellM)).toFloat() }
    private val ky = FloatArray(ny) { (2 * PI * frequency(it, ny) / (ny * cellM)).toFloat() }

    // Per mode, row-major (y * nx + x): exact one-step propagator and the
    // finger's force envelope, all real because they depend on |k| only.
    private val pHH = FloatArray(size)
    private val pHV = FloatArray(size)
    private val pVH = FloatArray(size)
    private val pVV = FloatArray(size)
    private val resolved = FloatArray(size)
    private val forceEnvelope = FloatArray(size)

    // State: height (m) and vertical speed (m/s) spectra.
    private val hRe = FloatArray(size)
    private val hIm = FloatArray(size)
    private val vRe = FloatArray(size)
    private val vIm = FloatArray(size)

    // Scratch and output.
    private val fft = Fft2d(nx, ny)
    private val workRe = FloatArray(size)
    private val workIm = FloatArray(size)
    private val colCos = FloatArray(nx)
    private val colSin = FloatArray(nx)
    private val rowCos = FloatArray(ny)
    private val rowSin = FloatArray(ny)
    private val gradX = FloatArray(visibleX * visibleY)
    private val gradY = FloatArray(visibleX * visibleY)
    private val pixels = IntArray(visibleX * visibleY)
    private val overlay = Bitmap.createBitmap(visibleX, visibleY, Bitmap.Config.ARGB_8888)
    private val overlayRect = RectF(0f, 0f, visibleX * cellPx, visibleY * cellPx)
    private val overlayPaint = Paint(Paint.FILTER_BITMAP_FLAG)

    private val stepS = 1f / config.waterStepsPerSecond
    private var accumulatorS = 0f

    /** True while the surface moves; false means flat and costs nothing. */
    var active = false
        private set

    init {
        val g = config.gravity.toDouble()
        val sigma = config.surfaceTensionOverDensity.toDouble()
        val nu = config.kinematicViscosity.toDouble()
        val dt = stepS.toDouble()
        val kCut = config.waterResolvedFraction * PI / cellM
        val s = config.fingerRadiusM.toDouble()
        // FT of a Gaussian pressure patch, divided by the cell area for the
        // unnormalised discrete transform; times -(k / rho) dt for the force.
        val amplitude = config.fingerPressurePa * 2 * PI * s * s / (cellM.toDouble() * cellM)
        for (y in 0 until ny) for (x in 0 until nx) {
            val i = y * nx + x
            val k = sqrt((kx[x] * kx[x] + ky[y] * ky[y]).toDouble())
            val omega0Sq = g * k + sigma * k * k * k
            val beta = config.filmDamping + 2 * nu * k * k
            val w = sqrt(max(omega0Sq - beta * beta, 1e-12))
            val e = exp(-beta * dt)
            val c = cos(w * dt)
            val sn = sin(w * dt)
            // Damped oscillator h'' + 2 beta h' + omega0^2 h = 0, exactly:
            pHH[i] = (e * (c + beta * sn / w)).toFloat()
            pHV[i] = (e * (sn / w)).toFloat()
            pVH[i] = (-e * (w + beta * beta / w) * sn).toFloat()
            pVV[i] = (e * (c - beta * sn / w)).toFloat()
            val r = exp(-(k / kCut).let { it * it * it * it })
            resolved[i] = r.toFloat()
            forceEnvelope[i] = (-(k / RHO) * amplitude * exp(-0.5 * k * k * s * s) * r * dt).toFloat()
        }
        // The mean water level never changes.
        pHH[0] = 0f; pHV[0] = 0f; pVH[0] = 0f; pVV[0] = 0f; forceEnvelope[0] = 0f
    }

    /** Advance by [dtS] seconds; [finger] is the touch in screen pixels, if any. */
    fun update(dtS: Float, finger: Vec2?) {
        if (finger == null && !active) {
            accumulatorS = 0f
            return
        }
        accumulatorS += dtS
        var steps = (accumulatorS / stepS).toInt()
        if (steps > config.waterMaxStepsPerFrame) {
            steps = config.waterMaxStepsPerFrame
            accumulatorS = 0f // Drop the backlog rather than fall behind.
        } else {
            accumulatorS -= steps * stepS
        }
        if (steps == 0) return
        if (finger != null) fingerPhase(finger)
        repeat(steps) {
            for (i in 0 until size) {
                if (finger != null) {
                    // Pressure p accelerates mode k by -(k / rho) p.
                    val y = i / nx
                    val x = i - y * nx
                    val re = colCos[x] * rowCos[y] - colSin[x] * rowSin[y]
                    val im = -(colCos[x] * rowSin[y] + colSin[x] * rowCos[y])
                    vRe[i] += forceEnvelope[i] * re
                    vIm[i] += forceEnvelope[i] * im
                }
                val hr = hRe[i]
                val hi = hIm[i]
                hRe[i] = pHH[i] * hr + pHV[i] * vRe[i]
                hIm[i] = pHH[i] * hi + pHV[i] * vIm[i]
                vRe[i] = pVH[i] * hr + pVV[i] * vRe[i]
                vIm[i] = pVH[i] * hi + pVV[i] * vIm[i]
            }
        }
        active = true
        val slope = renderFields()
        if (finger == null && slope < config.waterRestSlope) {
            // Flat to the eye: stop simulating until the next touch.
            hRe.fill(0f); hIm.fill(0f); vRe.fill(0f); vIm.fill(0f)
            active = false
        }
    }

    /** exp(-i k . x0) for the finger at x0, as separable row and column factors. */
    private fun fingerPhase(finger: Vec2) {
        val x0 = (finger.x / cellPx + padX) * cellM
        val y0 = (finger.y / cellPx + padY) * cellM
        for (x in 0 until nx) { colCos[x] = cos(kx[x] * x0); colSin[x] = sin(kx[x] * x0) }
        for (y in 0 until ny) { rowCos[y] = cos(ky[y] * y0); rowSin[y] = sin(ky[y] * y0) }
    }

    /** Slope and light overlay for this frame; returns the largest slope. */
    private fun renderFields(): Float {
        for (i in 0 until size) {
            workRe[i] = hRe[i] * resolved[i]
            workIm[i] = hIm[i] * resolved[i]
        }
        fft.inverse(workRe, workIm)
        val norm = 1f / size // workRe / size is now the height in metres.

        val optics = 1f - 1f / config.refractiveIndex
        val causticGain = -config.floorDepthM * optics * config.causticStrength
        val twoDx = 2f * cellM
        val dx2 = cellM * cellM
        var maxSlope = 0f
        for (j in 0 until visibleY) {
            val gy = j + padY
            val up = (if (gy > 0) gy - 1 else gy) * nx
            val down = (if (gy < ny - 1) gy + 1 else gy) * nx
            val row = gy * nx
            for (i in 0 until visibleX) {
                val gx = i + padX
                val left = if (gx > 0) gx - 1 else gx
                val right = if (gx < nx - 1) gx + 1 else gx
                val c = workRe[row + gx] * norm
                val l = workRe[row + left] * norm
                val r = workRe[row + right] * norm
                val u = workRe[up + gx] * norm
                val d = workRe[down + gx] * norm
                val sx = (r - l) / twoDx
                val sy = (d - u) / twoDx
                val laplacian = (l + r + u + d - 4f * c) / dx2
                val o = j * visibleX + i
                gradX[o] = sx
                gradY[o] = sy
                maxSlope = max(maxSlope, max(abs(sx), abs(sy)))

                val glint = sx * config.lightDirectionX + sy * config.lightDirectionY
                val light = (laplacian * causticGain + glint * config.glintStrength) * 255f
                pixels[o] = if (light > 0f) {
                    (min(light, config.lightMaxAlpha).toInt() shl 24) or config.lightColor
                } else {
                    (min(-light, config.shadeMaxAlpha).toInt() shl 24) or config.shadeColor
                }
            }
        }
        overlay.setPixels(pixels, 0, visibleX, 0, 0, visibleX, visibleY)
        return maxSlope
    }

    /**
     * Apparent shift, in pixels, of something [depthM] below the surface at
     * screen point ([x], [y]); written to [out]. Returns false when calm.
     */
    fun refraction(x: Float, y: Float, depthM: Float, out: FloatArray): Boolean {
        if (!active) return false
        val i = (x / cellPx).toInt().coerceIn(0, visibleX - 1)
        val j = (y / cellPx).toInt().coerceIn(0, visibleY - 1)
        val shift = depthM * (1f - 1f / config.refractiveIndex) * pxPerM
        out[0] = -gradX[j * visibleX + i] * shift
        out[1] = -gradY[j * visibleX + i] * shift
        return true
    }

    /** Draw the caustics and glints over the pond. */
    fun draw(canvas: Canvas) {
        if (active) canvas.drawBitmap(overlay, null, overlayRect, overlayPaint)
    }

    private companion object {
        const val RHO = 1000.0 // Density of water, kg/m^3.

        /** numpy's fftfreq numerator: 0, 1, ..., n/2 - 1, -n/2, ..., -1. */
        fun frequency(i: Int, n: Int) = if (i < n / 2) i else i - n
    }
}
