package io.github.markspan.koipond

import android.app.WallpaperManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.service.wallpaper.WallpaperService
import android.view.Choreographer
import android.view.MotionEvent
import android.view.Surface
import android.view.SurfaceHolder
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * The koi pond as a live wallpaper, for the lock screen and home screen.
 *
 * Each [Engine] owns a render thread. Frames are paced by the display's
 * vsync through [Choreographer] and drawn with a hardware-accelerated
 * canvas. Nothing runs while the wallpaper is not visible.
 */
class KoiWallpaperService : WallpaperService() {

    override fun onCreateEngine(): Engine = KoiEngine()

    private inner class KoiEngine : Engine() {
        private val thread = HandlerThread("koi-render").apply { start() }
        private val handler = Handler(thread.looper)

        // Render-thread state.
        private lateinit var choreographer: Choreographer
        private var pond: Pond? = null
        private var surfaceWidth = 0
        private var surfaceHeight = 0
        private var running = false
        private var lastFrameNanos = 0L

        // Touch state, written on the main thread and read on the render thread.
        @Volatile private var fingerX = 0f
        @Volatile private var fingerY = 0f
        @Volatile private var fingerDown = false
        @Volatile private var tapUntilMs = 0L

        private val frameCallback = object : Choreographer.FrameCallback {
            override fun doFrame(frameTimeNanos: Long) {
                if (!running) return
                val dtS = if (lastFrameNanos == 0L) 1f / 60f
                else ((frameTimeNanos - lastFrameNanos) / 1e9f).coerceIn(0f, 0.1f)
                lastFrameNanos = frameTimeNanos
                renderFrame(dtS)
                choreographer.postFrameCallback(this)
            }
        }

        override fun onCreate(surfaceHolder: SurfaceHolder) {
            super.onCreate(surfaceHolder)
            setTouchEventsEnabled(true) // The home screen passes touches; the lock screen does not.
            handler.post { choreographer = Choreographer.getInstance() }
        }

        override fun onSurfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
            super.onSurfaceChanged(holder, format, width, height)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                // Ask for a calm 60 Hz instead of 90/120 Hz: smoother and kinder to the battery.
                holder.surface.setFrameRate(PondConfig().frameRate, Surface.FRAME_RATE_COMPATIBILITY_DEFAULT)
            }
            handler.post {
                if (width != surfaceWidth || height != surfaceHeight || pond == null) {
                    surfaceWidth = width
                    surfaceHeight = height
                    pond = Pond(width, height)
                }
                renderFrame(0f) // Show the pond at once, even before animating.
            }
        }

        override fun onVisibilityChanged(visible: Boolean) {
            handler.post {
                running = visible
                lastFrameNanos = 0L
                choreographer.removeFrameCallback(frameCallback)
                if (visible) choreographer.postFrameCallback(frameCallback)
            }
        }

        override fun onSurfaceDestroyed(holder: SurfaceHolder) {
            // The surface must not be drawn on after this returns.
            val stopped = CountDownLatch(1)
            handler.post {
                running = false
                choreographer.removeFrameCallback(frameCallback)
                stopped.countDown()
            }
            stopped.await(1, TimeUnit.SECONDS)
            super.onSurfaceDestroyed(holder)
        }

        override fun onDestroy() {
            super.onDestroy()
            handler.post { pond = null }
            thread.quitSafely()
        }

        override fun onTouchEvent(event: MotionEvent) {
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                    fingerX = event.x
                    fingerY = event.y
                    fingerDown = true
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> fingerDown = false
            }
        }

        /** Launchers report taps on empty home screen space as a command. */
        override fun onCommand(action: String, x: Int, y: Int, z: Int, extras: Bundle?, resultRequested: Boolean): Bundle? {
            if (action == WallpaperManager.COMMAND_TAP) {
                fingerX = x.toFloat()
                fingerY = y.toFloat()
                tapUntilMs = SystemClock.uptimeMillis() + TAP_PRESS_MS
            }
            return super.onCommand(action, x, y, z, extras, resultRequested)
        }

        private fun currentFinger(): Vec2? {
            val pressed = fingerDown || SystemClock.uptimeMillis() < tapUntilMs
            return if (pressed) Vec2(fingerX, fingerY) else null
        }

        private fun renderFrame(dtS: Float) {
            val pond = pond ?: return
            if (dtS > 0f) pond.step(dtS, currentFinger())
            val holder = surfaceHolder
            val canvas = try {
                holder.lockHardwareCanvas()
            } catch (e: IllegalStateException) {
                null // The surface went away between frames.
            } ?: return
            try {
                pond.draw(canvas)
            } finally {
                holder.unlockCanvasAndPost(canvas)
            }
        }
    }

    private companion object {
        /** How long a launcher tap presses on the water. */
        const val TAP_PRESS_MS = 120L
    }
}
