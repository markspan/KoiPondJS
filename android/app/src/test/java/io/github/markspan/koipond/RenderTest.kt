package io.github.markspan.koipond

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode
import java.io.File
import kotlin.random.Random

/**
 * Renders skins and a pond frame with Android's real graphics (Robolectric
 * native mode), checks them, and saves PNGs to build/render-previews for
 * looking at.
 */
@RunWith(RobolectricTestRunner::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [34])
class RenderTest {
    private val previews = File("build/render-previews").apply { mkdirs() }

    private fun save(bitmap: Bitmap, name: String) {
        File(previews, "$name.png").outputStream().use { bitmap.compress(Bitmap.CompressFormat.PNG, 100, it) }
    }

    @Test
    fun everyVarietyHasASolidBody() {
        val rng = Random(1)
        // Body alpha at points well inside the body, from behind the head to near the tail.
        val probes = listOf(50, 80, 110, 140)
        val report = VARIETIES.associate { variety ->
            val skin = SkinPainter(220, 80).paint(variety, rng)
            save(skin, "skin-" + variety.name.replace(' ', '_'))
            variety.name to probes.map { x -> Color.alpha(skin.getPixel(x, 40)) }
        }
        val failures = report.filterValues { alphas -> alphas.any { it != 255 } }
        assertTrue("Bodies not solid (alpha at x = $probes): $failures", failures.isEmpty())
    }

    @Test
    fun pondFrameRenders() {
        val frame = Bitmap.createBitmap(1080, 2400, Bitmap.Config.ARGB_8888)
        val pond = Pond(1080, 2400)
        repeat(240) { pond.step(1f / 60f, null) } // Four seconds: every koi faded in.
        pond.draw(Canvas(frame))
        save(frame, "pond")
        // A touch: ripples and a stirred school.
        repeat(20) { pond.step(1f / 60f, Vec2(540f, 1200f)) }
        repeat(30) { pond.step(1f / 60f, null) }
        pond.draw(Canvas(frame))
        save(frame, "pond-touched")
    }
}
