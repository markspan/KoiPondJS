# Koi Pond live wallpaper (Android)

The koi pond from [`python/`](../python) as an Android live wallpaper, for the lock screen and home screen. It's written for Pixel phones and works on any phone with Android 8 or newer.

## Install

1. On your phone, open the [latest build](https://github.com/markspan/KoiPondJS/releases/tag/wallpaper-latest) and download `KoiPond.apk`.
2. Open the downloaded file and allow your browser to install apps when Android asks. This is called installing from unknown sources.
3. Open the **Koi Pond** app. It shows the wallpaper preview.
4. Tap **Set wallpaper** and choose **Home and lock screen**.

On a Pixel you can also find it later under **Wallpaper & style** > **More wallpapers** > **Live wallpapers**.

New builds install over the old one, since every build is signed with the same key.

## What it does

- **Koi:** the ten painted varieties of the Python version. Their skins bend along the swimming spine, as in the original Processing sketch.
- **Calm by default:** the koi glide slowly. On the **home screen**, touching the water stirs the pond: nearby fish bolt away, and everyone settles again over about 15 seconds.
- **Water:** the same physically based ripples as the Python version, with caustics on the floor, glints, and fish refracted by the waves.
- **Lock screen:** Android keeps touches on the lock screen for itself (unlocking, notifications), so there the koi simply swim calmly.
- **Battery:** nothing runs while the wallpaper is hidden. The display is asked for a steady 60 frames per second, and the water costs nothing while calm. The always-on display can't show live wallpapers.

## Build it yourself

GitHub Actions builds the APK on every push to `android/` and publishes it as the `wallpaper-latest` pre-release ([workflow](../.github/workflows/android-wallpaper.yml)). To build locally you need JDK 17 and the Android SDK:

```bash
./gradlew assembleRelease
```

The APK appears in `app/build/outputs/apk/release/`.

## Code

All code is in `app/src/main/java/io/github/markspan/koipond/`:

| File | What it holds |
|---|---|
| `PondConfig.kt` | Every tunable setting, matching `PondConfig` in the Python version |
| `Skins.kt` | The koi varieties and the procedural skin painter |
| `Koi.kt` | The swimming body (flagellum) and the steering behaviour |
| `WaterSurface.kt`, `Fft.kt` | The spectral water simulation and its optics |
| `Pond.kt` | Scenery, the school, and the stir level |
| `KoiWallpaperService.kt` | The wallpaper engine: render thread, frame pacing, touches |
| `SetWallpaperActivity.kt` | The app icon, which opens the wallpaper preview |

`debug.keystore` is committed on purpose. It's a standard debug key with the public password `android`, used so that updates install over each other. It isn't a secret.
