package io.github.markspan.koipond

import android.app.Activity
import android.app.WallpaperManager
import android.content.ActivityNotFoundException
import android.content.ComponentName
import android.content.Intent
import android.os.Bundle

/**
 * The app icon's only job: open the system preview of the koi wallpaper,
 * where "Set wallpaper" applies it to the lock screen and home screen.
 */
class SetWallpaperActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val preview = Intent(WallpaperManager.ACTION_CHANGE_LIVE_WALLPAPER).putExtra(
            WallpaperManager.EXTRA_LIVE_WALLPAPER_COMPONENT,
            ComponentName(this, KoiWallpaperService::class.java),
        )
        try {
            startActivity(preview)
        } catch (e: ActivityNotFoundException) {
            // Some launchers lack the direct preview; fall back to the chooser.
            startActivity(Intent(WallpaperManager.ACTION_LIVE_WALLPAPER_CHOOSER))
        }
        finish()
    }
}
