package io.github.markspan.koipond

import kotlin.math.cos
import kotlin.math.sin

/**
 * In-place radix-2 complex FFT of one power-of-two size.
 *
 * Sign convention as numpy: forward uses exp(-2 pi i k n / N), inverse uses
 * exp(+2 pi i k n / N) and is NOT normalised (the caller divides by N).
 */
class Fft(private val n: Int) {
    private val cosTable = FloatArray(n / 2) { cos(2.0 * Math.PI * it / n).toFloat() }
    private val sinTable = FloatArray(n / 2) { sin(2.0 * Math.PI * it / n).toFloat() }
    private val reversed = IntArray(n)

    init {
        require(n >= 2 && n and (n - 1) == 0) { "FFT size must be a power of two, got $n" }
        val bits = Integer.numberOfTrailingZeros(n)
        for (i in 0 until n) reversed[i] = Integer.reverse(i) ushr (32 - bits)
    }

    /** Transform `re[offset until offset + n]`, `im[...]` in place. */
    fun transform(re: FloatArray, im: FloatArray, offset: Int, inverse: Boolean) {
        for (i in 0 until n) {
            val j = reversed[i]
            if (j > i) {
                val a = offset + i
                val b = offset + j
                var t = re[a]; re[a] = re[b]; re[b] = t
                t = im[a]; im[a] = im[b]; im[b] = t
            }
        }
        val sign = if (inverse) 1f else -1f
        var size = 2
        while (size <= n) {
            val half = size / 2
            val step = n / size
            var start = 0
            while (start < n) {
                for (k in 0 until half) {
                    val wr = cosTable[k * step]
                    val wi = sign * sinTable[k * step]
                    val a = offset + start + k
                    val b = a + half
                    val tr = wr * re[b] - wi * im[b]
                    val ti = wr * im[b] + wi * re[b]
                    re[b] = re[a] - tr
                    im[b] = im[a] - ti
                    re[a] += tr
                    im[a] += ti
                }
                start += size
            }
            size *= 2
        }
    }
}

/** 2-D complex FFT on row-major arrays of `ny` rows by `nx` columns. */
class Fft2d(private val nx: Int, private val ny: Int) {
    private val rows = Fft(nx)
    private val cols = Fft(ny)
    private val colRe = FloatArray(ny)
    private val colIm = FloatArray(ny)

    /** Unnormalised inverse transform, in place. */
    fun inverse(re: FloatArray, im: FloatArray) {
        for (y in 0 until ny) rows.transform(re, im, y * nx, inverse = true)
        for (x in 0 until nx) {
            for (y in 0 until ny) {
                colRe[y] = re[y * nx + x]
                colIm[y] = im[y * nx + x]
            }
            cols.transform(colRe, colIm, 0, inverse = true)
            for (y in 0 until ny) {
                re[y * nx + x] = colRe[y]
                im[y * nx + x] = colIm[y]
            }
        }
    }
}
