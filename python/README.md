# KoiPondPy

A koi pond in Python / pygame for Android (**Pydroid 3**) and desktop.

It ports the behaviour of Ricardo Sánchez's Processing sketch "Koi Pond" (2009, [nardove/p5Kois](https://github.com/nardove/p5Kois)), which [KoiPondJS](https://github.com/markspan/KoiPondJS) borrowed its fish body from.

## What you see

- **Ten koi varieties:** Kohaku, Taisho Sanke, Showa, Tancho, Orenji Ogon, Ki Bekko, Aka Bekko, Asagi, Hariwake and Soragoi. Each fish's skin is painted when it is born, so every fish has its own markings.
- **Swimming bodies:** the skin is cut into strips that follow a spine of nodes. A neck "muscle" sends a wave down to the tail, and it beats faster when the fish swims faster.
- **Wandering and scattering:** fish wander slowly. Touching the water makes nearby fish bolt away, and their burst of speed wears off over about two seconds.
- **Depth:** a new koi arrives every half second, up to 60. Older fish swim deeper and look darker. New fish fade in.

## Run in Pydroid 3

1. Copy `koipond.py` to your phone.
2. In Pydroid: menu, **Pip**, search `pygame`, **Install**.
3. Open `koipond.py` and press **Run**.

Touch and drag to scare the koi. Press Back to quit.

## Run on a desktop

```bash
pip install pygame
```

```bash
python koipond.py
```

Press Escape to quit.

## Skin images

Without a `skins` folder, every fish gets a painted skin in one of the ten varieties above.

To use the original skin images instead, download `skin-0.png` to `skin-9.png` from Ricardo Sánchez's [p5Kois](https://github.com/nardove/p5Kois/tree/master/koi_pond_v10/data) and put them in a `skins` folder next to `koipond.py`. They are not included here because p5Kois has no license, so they remain his work.

You can also add your own PNGs: a top view of a koi, head pointing left, on a transparent background.

## Tuning

Every setting is in the `PondConfig` dataclass at the top of the file, including:

- the number of koi
- the depth brightness range
- the wander and flee strengths
- ripples

## Speed

Bending 60 fish means drawing 600 rotated skin strips every frame. By default the pond hands this to the phone's GPU through pygame's SDL2 renderer, which also applies each fish's depth shading and fade-in. The pond is then drawn at full screen resolution, and the CPU only runs the simulation.

If your pygame build has no SDL2 renderer, the pond falls back to drawing with the CPU at half resolution (`render_scale`), which SDL stretches to fill the screen.

Two settings in `PondConfig` control this:

- `renderer`: `"auto"` (default), `"gpu"` or `"software"`.
- `render_scale`: the CPU fallback's drawing resolution, 0.5 by default. Use 1.0 for full sharpness on a fast device.

If it is still slow on your phone, lower `max_koi` or `body_segments`.
