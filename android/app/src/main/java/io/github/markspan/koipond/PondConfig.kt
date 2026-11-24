package io.github.markspan.koipond

/**
 * All tunable constants of the pond in one place.
 *
 * This mirrors `PondConfig` in the Python version (`python/koipond.py`).
 * Distances and speeds are in pixels of the original 900 x 450 Processing
 * sketch, per frame at 60 frames per second; [Pond] multiplies them by a
 * screen scale and by the real frame time, so the pond looks and moves the
 * same on any screen and at any refresh rate.
 */
data class PondConfig(
    // Scene
    val maxKoi: Int = 60,
    /** Start with every koi present (fading in) instead of one by one. */
    val startWithFullPond: Boolean = true,
    val birthIntervalMs: Float = 500f,
    val referenceShortSide: Float = 450f,
    /** Frame rate requested from the display; lower saves battery. */
    val frameRate: Float = 60f,

    // Fish body
    val skinWidth: Int = 220, // Reference skin texture size.
    val skinHeight: Int = 80,
    val minSkinScale: Float = 0.5f,
    val maxSkinScale: Float = 1.0f,
    val bodySegments: Int = 12,
    val muscleRangeDeg: Float = 6f,
    val muscleRatePerSpeed: Float = 0.06f,

    // Depth: brightness (0..255) and depth below the surface, from the
    // oldest (deepest) to the newest (highest) koi.
    val depthBrightnessDeep: Int = 40,
    val depthBrightnessShallow: Int = 210,
    val fishDepthDeepM: Float = 0.30f,
    val fishDepthShallowM: Float = 0.05f,
    val fadeInPerFrame: Float = 2f,

    // Swimming: calm until the water is touched. A touch stirs the pond;
    // the stir fades with time constant stirFadeS.
    val calmSpeedMin: Float = 0.25f,
    val calmSpeedMax: Float = 0.6f,
    val stirredSpeedMin: Float = 0.8f,
    val stirredSpeedMax: Float = 1.9f,
    val calmForce: Float = 0.04f,
    val stirredForce: Float = 0.2f,
    val calmWanderJitter: Float = 0.015f,
    val stirredWanderJitter: Float = 0.05f,
    val speedEase: Float = 0.02f,
    val stirFadeS: Float = 5f,
    val wanderRadius: Float = 5f,
    val wanderDistance: Float = 100f,

    // Fleeing from a touch
    val scareRadius: Float = 100f,
    val rushSpeedMin: Float = 3f,
    val rushSpeedMax: Float = 6f,
    val rushSlowdown: Float = 3f,
    val rushSteps: Int = 10,
    val rushStepMs: Float = 200f,

    // Water surface, with real physical constants. The screen's short side
    // shows pondShortSideM of real pond.
    val pondShortSideM: Float = 0.6f,
    /** Simulation grid; powers of two (FFT). The first is across the short side. */
    val waterGridShort: Int = 128,
    val waterGridLong: Int = 256,
    /** Share of the grid kept beyond each screen edge, so waves fade before wrapping. */
    val waterMargin: Float = 0.1f,
    val waterStepsPerSecond: Float = 60f,
    val waterMaxStepsPerFrame: Int = 6,
    val waterRestSlope: Float = 0.002f,
    val waterResolvedFraction: Float = 0.45f,
    val gravity: Float = 9.81f,
    val surfaceTensionOverDensity: Float = 7.28e-5f,
    val kinematicViscosity: Float = 1.0e-6f,
    val filmDamping: Float = 0.6f,
    val fingerPressurePa: Float = 60f,
    val fingerRadiusM: Float = 0.005f,
    val refractiveIndex: Float = 1.333f,
    val floorDepthM: Float = 0.35f,
    val lightDirectionX: Float = -0.6f,
    val lightDirectionY: Float = -0.8f,
    val causticStrength: Float = 1.6f,
    val glintStrength: Float = 0.8f,
    val lightColor: Int = 0xECF6FF,
    val shadeColor: Int = 0x000A18,
    val lightMaxAlpha: Float = 110f,
    val shadeMaxAlpha: Float = 70f,
)
