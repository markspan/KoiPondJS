"""Koi Pond: a Python / pygame koi pond for Pydroid 3 (Android) and desktop.

A port of the behaviour in Ricardo Sanchez's Processing sketch "Koi Pond"
(2009, https://github.com/nardove/p5Kois), the sketch that KoiPondJS
(https://github.com/markspan/KoiPondJS) borrowed its fish body from.

What it shows
-------------
Koi of ten real varieties (Kohaku, Taisho Sanke, Showa, Tancho, Ogon,
Ki Bekko, Aka Bekko, Asagi, Hariwake, Soragoi) are born one by one over a
dark pebble bed until the pond holds ``max_koi`` fish.

* Body: every fish is a chain of nodes ("flagellum"). An oscillating neck
  muscle bends the chain and the bend travels down to the tail. The muscle
  beats faster the faster the fish swims. The skin texture is cut into
  strips and each strip is drawn along one body segment, so the painted
  skin bends with the spine.
* Behaviour: fish wander (Reynolds' wander steering). Touching the water
  scares nearby fish: they flee from your finger with a burst of speed that
  slowly wears off.
* Depth: fish born earlier are drawn first and darker, so the pond looks
  deep. New fish fade in.

Speed
-----
Bending 60 fish means drawing 600 rotated skin strips per frame. By default
this is done by the GPU through SDL2's renderer (``pygame._sdl2.video``),
which also applies the depth tint and fade. Where that is unavailable, the
pond falls back to CPU drawing at reduced resolution. See ``renderer`` and
``render_scale`` in :class:`PondConfig`.

Skins
-----
Every fish gets its own procedurally painted skin, so no two fish look the
same and no image files are needed. If you have skin images of your own
(a top view of a koi, head pointing left, transparent background), put them
as PNG files in a ``skins`` folder next to this script and they are used
instead.

Running on Android (Pydroid 3)
------------------------------
1. Install pygame: Pydroid menu, Pip, search "pygame", Install.
2. Open this file and press Run. It starts full screen.

Controls
--------
* Touch and drag: scare the koi near your finger (and make ripples).
* Back button, Escape or closing the window: quit.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path

import pygame
import pygame.gfxdraw
from pygame.math import Vector2

Color = tuple[int, int, int]

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PondConfig:
    """All tunable constants in one place.

    Distances and speeds are in pixels of the original 900 x 450 sketch and
    are multiplied by :attr:`Pond.scale`, so the pond keeps its proportions
    on any screen.
    """

    # Scene
    max_koi: int = 60
    birth_interval_ms: int = 500
    fps: int = 60
    reference_short_side: float = 450.0  # Short side of the original sketch.
    # Window size in pixels, or None for full screen.
    window_size: tuple[int, int] | None = None
    # "auto" uses the GPU when this pygame supports it, else the CPU.
    # Force one with "gpu" or "software".
    renderer: str = "auto"
    # CPU renderer only: fraction of the native resolution the pond is drawn
    # at before SDL stretches it to the screen. 0.5 costs a quarter of the
    # work. The GPU renderer always draws at full resolution.
    render_scale: float = 0.5
    skins_folder: str = "skins"

    # Fish body
    skin_size: tuple[int, int] = (220, 80)  # Reference skin texture size.
    min_skin_scale: float = 0.5
    max_skin_scale: float = 1.0
    body_segments: int = 10  # Strips the skin is bent along.
    muscle_range_deg: float = 6.0
    muscle_rate_per_speed: float = 0.06  # Muscle phase per pixel/frame.

    # Depth: per-fish brightness from the oldest to the newest fish (0..255).
    # The original used 10..170; slightly brighter reads better on a phone.
    depth_brightness: tuple[int, int] = (40, 210)
    fade_in_per_frame: int = 2

    # Wandering (values from the original sketch)
    min_speed: float = 0.8
    max_speed: float = 1.9
    max_force: float = 0.2
    wander_radius: float = 5.0
    wander_distance: float = 100.0
    wander_jitter: float = 0.05

    # Fleeing from a touch
    scare_radius: float = 100.0
    rush_speed: tuple[float, float] = (3.0, 6.0)
    rush_slowdown: float = 3.0  # Speed lost over the whole rush.
    rush_steps: int = 10
    rush_step_ms: int = 200

    # Ripples on touch
    ripples: bool = True
    ripple_interval_ms: int = 90
    ripple_lifetime_ms: int = 1400
    ripple_max_radius: float = 70.0


CONFIG = PondConfig()


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def limit(vector: Vector2, max_length: float) -> Vector2:
    """Clamp ``vector`` in place to ``max_length`` and return it (p5 ``limit``)."""
    if vector.length_squared() > max_length * max_length:
        vector.scale_to_length(max_length)
    return vector


def mix(a: Color, b: Color, t: float) -> Color:
    """Linear blend of two RGB colours, ``t`` = 0 gives ``a``."""
    return tuple(round(x + (y - x) * t) for x, y in zip(a, b))  # type: ignore[return-value]


def bezier(
    start: tuple[float, float],
    control: tuple[float, float],
    end: tuple[float, float],
    steps: int = 10,
) -> list[tuple[float, float]]:
    """Points along a quadratic Bezier curve, both ends included."""
    points = []
    for i in range(steps + 1):
        t = i / steps
        u = 1.0 - t
        points.append(
            (
                u * u * start[0] + 2 * u * t * control[0] + t * t * end[0],
                u * u * start[1] + 2 * u * t * control[1] + t * t * end[1],
            )
        )
    return points


def rgba(color: Color, alpha: int) -> tuple[int, int, int, int]:
    return (*color, alpha)


WHITE: Color = (244, 242, 236)
BLACK: Color = (22, 20, 24)
RED: Color = (204, 38, 32)


# ---------------------------------------------------------------------------
# Koi varieties
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PatchSpec:
    """A kind of colour patch painted on the body.

    ``radius`` is a fraction of the skin height, ``x_range`` a fraction of
    the skin width (0 is the nose). Each patch is a cluster of ``blobs``
    overlapping discs, which gives the irregular koi markings.
    """

    color: Color
    count: tuple[int, int]
    radius: tuple[float, float]
    x_range: tuple[float, float] = (0.08, 0.66)
    blobs: tuple[int, int] = (3, 6)


@dataclass(frozen=True)
class HeadSpec:
    """A marking on the head: a round spot (Tancho) or a cap (Soragoi)."""

    color: Color
    radius: float  # Fraction of the skin height.
    x: float = 0.13  # Centre, fraction of the skin width.


@dataclass(frozen=True)
class Variety:
    """Colouring of one koi variety."""

    name: str
    base: Color
    fin: Color
    patches: tuple[PatchSpec, ...] = ()
    head: HeadSpec | None = None
    metallic: bool = False
    netted: bool = False  # Strong reticulated scale net (Asagi, Soragoi).


VARIETIES: tuple[Variety, ...] = (
    Variety("Kohaku", WHITE, (250, 236, 232), (PatchSpec(RED, (2, 4), (0.12, 0.2)),)),
    Variety(
        "Taisho Sanke",
        WHITE,
        (250, 238, 234),
        (
            PatchSpec(RED, (2, 3), (0.11, 0.18)),
            PatchSpec(BLACK, (3, 6), (0.04, 0.08), (0.18, 0.72), (2, 4)),
        ),
    ),
    Variety(
        "Showa",
        BLACK,
        (60, 56, 60),
        (
            PatchSpec(RED, (2, 4), (0.13, 0.2)),
            PatchSpec(WHITE, (1, 3), (0.08, 0.14)),
        ),
    ),
    Variety("Tancho", WHITE, (248, 244, 240), head=HeadSpec(RED, 0.13)),
    Variety("Orenji Ogon", (244, 138, 36), (250, 170, 90), metallic=True),
    Variety(
        "Ki Bekko",
        (250, 196, 28),
        (252, 214, 70),
        (PatchSpec(BLACK, (3, 6), (0.04, 0.08), (0.12, 0.7), (2, 4)),),
    ),
    Variety(
        "Aka Bekko",
        (188, 34, 34),
        (236, 170, 170),
        (PatchSpec(BLACK, (3, 6), (0.05, 0.1), (0.12, 0.7), (2, 5)),),
    ),
    Variety(
        "Asagi",
        (96, 128, 186),
        (186, 214, 240),
        (PatchSpec(RED, (2, 3), (0.1, 0.17)),),
        netted=True,
    ),
    Variety(
        "Hariwake",
        (230, 230, 226),
        (246, 222, 196),
        (PatchSpec((240, 136, 40), (2, 4), (0.11, 0.19)),),
        metallic=True,
    ),
    Variety(
        "Soragoi",
        (84, 94, 130),
        (176, 190, 220),
        head=HeadSpec((240, 206, 182), 0.3, x=0.05),
        netted=True,
    ),
)


# ---------------------------------------------------------------------------
# Skin painting
# ---------------------------------------------------------------------------


class SkinPainter:
    """Paints a top-view koi texture, head on the left, tail on the right.

    The layout matches the original skin images: the body runs from the
    nose to the tail root, the tail fin fills the right end and the
    pectoral fins reach the top and bottom edges.
    """

    NOSE_X = 0.02  # Body extent, as fractions of the width.
    TAIL_ROOT_X = 0.80
    BODY_HALF_WIDTH = 0.29  # Fraction of the height.
    PEDUNCLE_HALF_WIDTH = 0.05

    def __init__(self, width: int, height: int) -> None:
        self.w = width
        self.h = height
        self.cy = height / 2.0

    # -- geometry -----------------------------------------------------------

    def half_width(self, x: float) -> float:
        """Body half-width at horizontal pixel position ``x``."""
        x0, x1 = self.NOSE_X * self.w, self.TAIL_ROOT_X * self.w
        t = (x - x0) / (x1 - x0)
        if t <= 0.0 or t > 1.0:
            return 0.0
        widest = self.BODY_HALF_WIDTH * self.h
        if t < 0.25:  # Rounded head.
            return widest * math.sqrt(max(0.0, 1.0 - ((0.25 - t) / 0.25) ** 2))
        narrow = self.PEDUNCLE_HALF_WIDTH * self.h
        taper = math.cos((t - 0.25) / 0.75 * math.pi / 2.0) ** 1.2
        return narrow + (widest - narrow) * taper

    def body_outline(self, y_scale: float = 1.0) -> list[tuple[float, float]]:
        """Closed body outline; ``y_scale`` < 1 gives a slimmer inner copy."""
        x0, x1 = self.NOSE_X * self.w, self.TAIL_ROOT_X * self.w
        steps = 40
        xs = [x0 + (x1 - x0) * i / steps for i in range(steps + 1)]
        top = [(x, self.cy - self.half_width(x) * y_scale) for x in xs]
        bottom = [(x, self.cy + self.half_width(x) * y_scale) for x in reversed(xs)]
        return top + bottom

    # -- painting -----------------------------------------------------------

    def paint(self, variety: Variety, rng: random.Random) -> pygame.Surface:
        """Return a new texture for one fish of ``variety``."""
        skin = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        self._paint_fins(skin, variety.fin)
        skin.blit(self._paint_body(variety, rng), (0, 0))
        self._paint_details(skin)
        return skin

    def _paint_body(self, variety: Variety, rng: random.Random) -> pygame.Surface:
        body = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        body.fill(rgba(variety.base, 255))

        # Soft shading: lighter along the spine, darker at the flanks.
        shade = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        pygame.draw.polygon(shade, (0, 0, 0, 60), self.body_outline())
        for i, y_scale in enumerate((0.9, 0.75, 0.6, 0.45, 0.3)):
            alpha = 50 - i * 6
            pygame.draw.polygon(shade, (255, 255, 255, alpha), self.body_outline(y_scale))
        body.blit(shade, (0, 0))

        for spec in variety.patches:
            self._paint_patches(body, spec, rng)
        if variety.head:
            self._paint_head(body, variety.head)
        self._paint_scales(body, variety)
        if variety.metallic:
            self._paint_sheen(body)

        # Keep only what lies inside the body outline.
        mask = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        pygame.draw.polygon(mask, (255, 255, 255, 255), self.body_outline())
        body.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        return body

    def _paint_patches(self, body: pygame.Surface, spec: PatchSpec, rng: random.Random) -> None:
        layer = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        discs: list[tuple[float, float, float]] = []
        for _ in range(rng.randint(*spec.count)):
            seed_x = rng.uniform(*spec.x_range) * self.w
            spread = self.half_width(seed_x) * 0.7
            seed_y = self.cy + rng.uniform(-spread, spread)
            for _ in range(rng.randint(*spec.blobs)):
                r = rng.uniform(*spec.radius) * self.h
                discs.append(
                    (seed_x + rng.uniform(-r, r) * 1.4, seed_y + rng.uniform(-r, r) * 0.8, r)
                )
        # Faint halo first, then the solid cores, for soft-edged markings.
        for x, y, r in discs:
            pygame.draw.circle(layer, rgba(spec.color, 110), (x, y), r * 1.2)
        for x, y, r in discs:
            pygame.draw.circle(layer, rgba(spec.color, 255), (x, y), r)
        body.blit(layer, (0, 0))

    def _paint_head(self, body: pygame.Surface, head: HeadSpec) -> None:
        layer = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        center = (head.x * self.w, self.cy)
        radius = head.radius * self.h
        pygame.draw.circle(layer, rgba(head.color, 120), center, radius * 1.12)
        pygame.draw.circle(layer, rgba(head.color, 255), center, radius)
        body.blit(layer, (0, 0))

    def _paint_scales(self, body: pygame.Surface, variety: Variety) -> None:
        """Rows of arcs bulging towards the tail, like overlapping scales."""
        layer = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        dark_base = sum(variety.base) < 300
        if variety.netted:
            color = (235, 240, 250, 90)
        elif dark_base:
            color = (255, 255, 255, 40)
        else:
            color = (0, 0, 0, 34)
        r = max(2.0, 0.055 * self.h)
        start_x = (self.NOSE_X + 0.2) * self.w  # No scales on the head.
        row = 0
        y = self.cy - self.BODY_HALF_WIDTH * self.h
        while y < self.cy + self.BODY_HALF_WIDTH * self.h:
            x = start_x + (r * 0.8 if row % 2 else 0.0)
            while x < self.TAIL_ROOT_X * self.w:
                rect = pygame.Rect(0, 0, 2 * r, 2 * r)
                rect.center = (round(x), round(y))
                pygame.draw.arc(layer, color, rect, -math.pi / 2, math.pi / 2, 1)
                x += r * 1.6
            y += r * 0.95
            row += 1
        body.blit(layer, (0, 0))

    def _paint_sheen(self, body: pygame.Surface) -> None:
        """Metallic varieties get a bright band along the back."""
        layer = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        for i in range(4):
            t = 0.14 - i * 0.03
            rect = pygame.Rect(0, 0, 0.62 * self.w * (1 - i * 0.12), 2 * t * self.h)
            rect.center = (round(0.38 * self.w), round(self.cy))
            pygame.draw.ellipse(layer, (255, 255, 240, 34), rect)
        body.blit(layer, (0, 0))

    def _paint_fins(self, skin: pygame.Surface, color: Color) -> None:
        w, h, cy = self.w, self.h, self.cy
        fins = pygame.Surface((w, h), pygame.SRCALPHA)
        rays = pygame.Surface((w, h), pygame.SRCALPHA)
        ray_color = (*mix(color, (90, 90, 90), 0.5), 110)

        # Tail fin, with a notch in the middle.
        root = self.TAIL_ROOT_X * w - 0.02 * w
        tail = [
            (root, cy - 0.05 * h),
            (0.985 * w, cy - 0.23 * h),
            (0.93 * w, cy),
            (0.985 * w, cy + 0.23 * h),
            (root, cy + 0.05 * h),
        ]
        pygame.draw.polygon(fins, rgba(color, 200), tail)
        for k in range(-3, 4):
            tip_y = cy + k * 0.065 * h
            tip_x = 0.97 * w - abs(k) * 0.004 * w if k else 0.94 * w
            pygame.draw.line(rays, ray_color, (root, cy), (tip_x, tip_y))

        # Pectoral fins, top and bottom, swept back.
        for side in (-1, 1):
            base_front = 0.19 * w
            base_back = 0.29 * w
            front = (base_front, cy + side * self.half_width(base_front) * 0.9)
            back = (base_back, cy + side * self.half_width(base_back) * 0.9)
            tip = (0.265 * w, cy + side * 0.48 * h)
            # Leaf shape: a curved leading edge out to the tip, a fuller
            # trailing edge back to the body.
            fin = bezier(front, (0.19 * w, cy + side * 0.42 * h), tip)
            fin += bezier(tip, (0.33 * w, cy + side * 0.44 * h), back)[1:]
            pygame.draw.polygon(fins, rgba(color, 200), fin)
            for k in range(4):
                ray_tip = fin[len(fin) // 2 - 3 + 2 * k]
                pygame.draw.line(rays, ray_color, front, ray_tip)

        fins.blit(rays, (0, 0))
        skin.blit(fins, (0, 0))

    def _paint_details(self, skin: pygame.Surface) -> None:
        """Outline and eyes on top of everything else."""
        outline = [(round(x), round(y)) for x, y in self.body_outline()]
        pygame.gfxdraw.aapolygon(skin, outline, (20, 20, 24, 170))
        eye_r = max(1, round(0.032 * self.h))
        eye_x = round(0.075 * self.w)
        for side in (-1, 1):
            eye_y = round(self.cy + side * 0.15 * self.h)
            pygame.gfxdraw.filled_circle(skin, eye_x, eye_y, eye_r, (12, 12, 16, 255))
            pygame.gfxdraw.aacircle(skin, eye_x, eye_y, eye_r, (12, 12, 16, 255))


class SkinLibrary:
    """Hands out skins: custom PNGs if present, painted varieties otherwise."""

    def __init__(self, folder: Path, config: PondConfig = CONFIG) -> None:
        self.config = config
        # (name, image) pairs; the file name stands in for the variety name.
        self.custom = [(p.stem, pygame.image.load(str(p))) for p in sorted(folder.glob("*.png"))]

    def make(self, scale: float, rng: random.Random) -> tuple[pygame.Surface, str]:
        """Return a fresh skin at ``scale`` and the variety name."""
        size = scale * rng.uniform(self.config.min_skin_scale, self.config.max_skin_scale)
        if self.custom:
            name, image = rng.choice(self.custom)
            source = image.convert_alpha()
            dims = (max(8, round(source.get_width() * size)), max(4, round(source.get_height() * size)))
            return pygame.transform.smoothscale(source, dims), name
        w, h = self.config.skin_size
        painter = SkinPainter(max(16, round(w * size)), max(8, round(h * size)))
        variety = rng.choice(VARIETIES)
        return painter.paint(variety, rng), variety.name



# ---------------------------------------------------------------------------
# Drawing back ends
# ---------------------------------------------------------------------------
#
# Bending a fish means drawing every skin strip rotated along its body
# segment: 60 fish x 10 strips = 600 rotated images per frame. The GPU does
# this almost for free, the CPU does not. ``GpuCanvas`` therefore draws
# through SDL2's hardware renderer; ``SoftwareCanvas`` is the fallback for
# pygame builds without it. Both offer the same small interface, so the
# simulation never needs to know which one is in use.


@dataclass(frozen=True)
class Segment:
    """Where one skin strip goes: its centre on screen and its angle.

    ``angle`` is in degrees, clockwise on screen (y points down), measured
    from the strip's own +x axis, which runs from head to tail.
    """

    center: Vector2
    angle: float


class Canvas:
    """Interface shared by the drawing back ends."""

    size: tuple[int, int]

    def set_scenery(self, background: pygame.Surface, shadow: pygame.Surface) -> None:
        """Set the pond floor (drawn first) and rim shadow (drawn last)."""
        raise NotImplementedError

    def add_skin(self, skin: pygame.Surface, segments: int, brightness: int) -> object:
        """Prepare ``skin`` for drawing in ``segments`` strips; returns a handle."""
        raise NotImplementedError

    def begin_frame(self) -> None:
        """Start a frame by drawing the pond floor."""
        raise NotImplementedError

    def draw_body(self, skin: object, segments: list[Segment], alpha: int) -> None:
        """Draw a fish: strip ``i`` of ``skin`` placed at ``segments[i]``."""
        raise NotImplementedError

    def draw_ripple(self, center: tuple[int, int], radius: float, alpha: int) -> None:
        raise NotImplementedError

    def end_frame(self) -> None:
        """Draw the rim shadow and show the frame."""
        raise NotImplementedError

    def screenshot(self) -> pygame.Surface:
        """A copy of the last frame, for tests and sharing."""
        raise NotImplementedError


def strip_rects(width: int, height: int, segments: int) -> list[pygame.Rect]:
    """Source rectangles cutting a skin into ``segments`` vertical strips.

    Strips overlap their neighbours by a pixel on each side so no seams
    show once they are rotated.
    """
    spacing = width / segments
    rects = []
    for i in range(segments):
        left = max(0, math.floor(i * spacing) - 1)
        right = min(width, math.ceil((i + 1) * spacing) + 1)
        rects.append(pygame.Rect(left, 0, right - left, height))
    return rects


def paint_ring(radius: int) -> pygame.Surface:
    """A soft white ring, scaled and faded by the GPU to draw ripples."""
    ring = pygame.Surface((2 * radius + 4, 2 * radius + 4), pygame.SRCALPHA)
    c = radius + 2
    pygame.gfxdraw.aacircle(ring, c, c, radius, (255, 255, 255, 255))
    pygame.gfxdraw.aacircle(ring, c, c, radius - 1, (255, 255, 255, 128))
    return ring


RIPPLE_COLOR: Color = (200, 220, 240)


@dataclass
class _GpuSkin:
    texture: object  # pygame._sdl2.video.Texture
    rects: list[pygame.Rect]
    brightness: int


class GpuCanvas(Canvas):
    """Draws with SDL2's hardware renderer (``pygame._sdl2.video``).

    Rotation, depth tint and fade are applied by the GPU while copying each
    strip, so a fish costs one tiny draw call per strip and textures never
    need to be modified. Everything runs at the screen's full resolution.
    """

    RING_RADIUS = 64

    def __init__(self, window_size: tuple[int, int] | None) -> None:
        from pygame._sdl2 import video  # Imported here: optional in pygame.

        self._video = video
        if window_size is None:
            self.window = video.Window("Koi Pond", fullscreen_desktop=True)
        else:
            self.window = video.Window("Koi Pond", size=window_size)
        try:
            self.renderer = video.Renderer(self.window, vsync=True)
        except (TypeError, pygame.error):  # Older pygame or no vsync support.
            self.renderer = video.Renderer(self.window)
        self.size = tuple(self.window.size)
        self._background = None
        self._shadow = None
        self._ring = video.Texture.from_surface(self.renderer, paint_ring(self.RING_RADIUS))

    def set_scenery(self, background: pygame.Surface, shadow: pygame.Surface) -> None:
        self._background = self._video.Texture.from_surface(self.renderer, background)
        self._shadow = self._video.Texture.from_surface(self.renderer, shadow)

    def add_skin(self, skin: pygame.Surface, segments: int, brightness: int) -> _GpuSkin:
        texture = self._video.Texture.from_surface(self.renderer, skin)
        rects = strip_rects(skin.get_width(), skin.get_height(), segments)
        return _GpuSkin(texture, rects, brightness)

    def begin_frame(self) -> None:
        self.renderer.draw_color = (0, 0, 0, 255)
        self.renderer.clear()
        if self._background is not None:
            self._background.draw()

    def draw_body(self, skin: _GpuSkin, segments: list[Segment], alpha: int) -> None:
        texture = skin.texture
        texture.color = (skin.brightness, skin.brightness, skin.brightness)
        texture.alpha = alpha
        for rect, segment in zip(skin.rects, segments):
            dest = pygame.Rect(0, 0, rect.width, rect.height)
            dest.center = (round(segment.center.x), round(segment.center.y))
            # SDL rotates clockwise around the destination centre.
            texture.draw(srcrect=rect, dstrect=dest, angle=segment.angle)

    def draw_ripple(self, center: tuple[int, int], radius: float, alpha: int) -> None:
        size = round(2 * radius * (self.RING_RADIUS + 2) / self.RING_RADIUS)
        dest = pygame.Rect(0, 0, size, size)
        dest.center = center
        self._ring.color = RIPPLE_COLOR
        self._ring.alpha = alpha
        self._ring.draw(dstrect=dest)

    def end_frame(self) -> None:
        if self._shadow is not None:
            self._shadow.draw()
        self.renderer.present()

    def screenshot(self) -> pygame.Surface:
        return self.renderer.to_surface()


@dataclass
class _SoftwareSkin:
    strips: list[pygame.Surface]


class SoftwareCanvas(Canvas):
    """Draws on an ordinary pygame display surface with the CPU.

    Rotating hundreds of strips per frame is the bottleneck here, so the
    pond is drawn at ``render_scale`` of the screen and, where pygame's
    SCALED mode is available, stretched to full screen by SDL.
    """

    def __init__(self, window_size: tuple[int, int] | None, render_scale: float) -> None:
        self.screen = self._open(window_size, render_scale)
        self.size = self.screen.get_size()
        self._background: pygame.Surface | None = None
        self._shadow: pygame.Surface | None = None

    @staticmethod
    def _open(window_size: tuple[int, int] | None, render_scale: float) -> pygame.Surface:
        if window_size is not None:
            return pygame.display.set_mode(window_size)
        info = pygame.display.Info()
        native = (info.current_w, info.current_h)
        if render_scale < 1.0 and min(native) > 0 and hasattr(pygame, "SCALED"):
            size = (round(native[0] * render_scale), round(native[1] * render_scale))
            try:
                return pygame.display.set_mode(size, pygame.FULLSCREEN | pygame.SCALED)
            except pygame.error:
                pass  # Fall back to drawing at the native resolution.
        # (0, 0) with FULLSCREEN takes the native resolution (the whole phone).
        return pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

    def set_scenery(self, background: pygame.Surface, shadow: pygame.Surface) -> None:
        self._background = background.convert()
        self._shadow = shadow.convert_alpha()

    def add_skin(self, skin: pygame.Surface, segments: int, brightness: int) -> _SoftwareSkin:
        # Bake the depth tint into a private copy; the CPU has no colour mod.
        tinted = skin.convert_alpha()
        tinted.fill((brightness, brightness, brightness, 255), special_flags=pygame.BLEND_RGBA_MULT)
        rects = strip_rects(tinted.get_width(), tinted.get_height(), segments)
        return _SoftwareSkin([tinted.subsurface(r).copy() for r in rects])

    def begin_frame(self) -> None:
        if self._background is not None:
            self.screen.blit(self._background, (0, 0))
        else:
            self.screen.fill((0, 0, 0))

    def draw_body(self, skin: _SoftwareSkin, segments: list[Segment], alpha: int) -> None:
        for strip, segment in zip(skin.strips, segments):
            # pygame rotates counter-clockwise, hence the minus sign.
            rotated = pygame.transform.rotate(strip, -segment.angle)
            if alpha < 255:
                rotated.set_alpha(alpha)
            center = (round(segment.center.x), round(segment.center.y))
            self.screen.blit(rotated, rotated.get_rect(center=center))

    def draw_ripple(self, center: tuple[int, int], radius: float, alpha: int) -> None:
        r = round(radius)
        if r > 1:
            x, y = center
            pygame.gfxdraw.aacircle(self.screen, x, y, r, (*RIPPLE_COLOR, alpha))
            pygame.gfxdraw.aacircle(self.screen, x, y, r + 1, (*RIPPLE_COLOR, alpha // 2))

    def end_frame(self) -> None:
        if self._shadow is not None:
            self.screen.blit(self._shadow, (0, 0))
        pygame.display.flip()

    def screenshot(self) -> pygame.Surface:
        return self.screen.copy()


def open_canvas(config: PondConfig) -> Canvas:
    """Open the window with the fastest back end this pygame supports."""
    if config.renderer in ("auto", "gpu"):
        try:
            return GpuCanvas(config.window_size)
        except (ImportError, AttributeError, pygame.error):
            if config.renderer == "gpu":
                raise
    return SoftwareCanvas(config.window_size, config.render_scale)


# ---------------------------------------------------------------------------
# Fish
# ---------------------------------------------------------------------------


class Flagellum:
    """A chain of nodes that swims like a fish body.

    Logic from levitated.com via the original sketch: the neck node swings
    with a sine "muscle" and every following node is dragged along at a
    fixed distance, so the wave travels down to the tail. Node positions are
    offsets from the fish's location; node 0 is the head.
    """

    def __init__(self, length: float, config: PondConfig = CONFIG) -> None:
        self.config = config
        self.spacing = length / config.body_segments
        self.muscle_phase = random.uniform(0.0, math.tau)
        self.muscle_rate = 0.0
        self.heading = random.uniform(0.0, math.tau)
        direction = Vector2(math.cos(self.heading), math.sin(self.heading))
        self.nodes = [-direction * self.spacing * n for n in range(config.body_segments + 1)]

    def move(self) -> None:
        """Swing the neck and drag the spine behind it."""
        head = Vector2(math.cos(self.heading), math.sin(self.heading))
        self.nodes[0] = head
        self.muscle_phase += self.muscle_rate
        neck = self.heading + math.radians(
            self.config.muscle_range_deg * math.sin(self.muscle_phase)
        )
        self.nodes[1] = head - Vector2(math.cos(neck), math.sin(neck)) * self.spacing
        for n in range(2, len(self.nodes)):
            pull = self.nodes[n] - self.nodes[n - 2]
            if pull.length_squared() == 0.0:
                pull = -head
            pull.scale_to_length(self.spacing)
            self.nodes[n] = self.nodes[n - 1] + pull

    def segments(self, origin: Vector2) -> list[Segment]:
        """Screen placement of every body segment, head first."""
        placed = []
        for a, b in zip(self.nodes, self.nodes[1:]):
            # A strip's +x (towards the tail) points from node i to node i+1.
            angle = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
            placed.append(Segment(origin + (a + b) * 0.5, angle))
        return placed


class Koi:
    """A wandering fish that bolts away from touches."""

    def __init__(
        self,
        skin: object,
        skin_width: int,
        variety: str,
        position: Vector2,
        scale: float,
        config: PondConfig = CONFIG,
    ) -> None:
        self.config = config
        self.scale = scale
        self.variety = variety
        self.skin = skin  # Handle from Canvas.add_skin.
        self.skin_width = skin_width
        self.body = Flagellum(skin_width, config)

        self.position = Vector2(position)
        self.velocity = Vector2()
        self.acceleration = Vector2()
        self.max_force = config.max_force * scale
        self.max_speed = self._cruise_speed()
        self.rush_speed = random.uniform(*config.rush_speed) * scale
        self.wander_theta = random.uniform(0.0, math.tau)
        self.opacity = 0

        # Rush state after a scare: counts down in fixed time steps.
        self.rush_step: int | None = None
        self.next_rush_tick = 0

    def _cruise_speed(self) -> float:
        return random.uniform(self.config.min_speed, self.config.max_speed) * self.scale

    # -- steering -----------------------------------------------------------

    def _steer(self, target: Vector2) -> Vector2:
        """Steering force towards ``target`` at full speed (desired minus velocity)."""
        desired = target - self.position
        if desired.length_squared() == 0.0:
            return Vector2()
        desired.scale_to_length(self.max_speed)
        return limit(desired - self.velocity, self.max_force)

    def wander(self) -> None:
        """Seek a point that drifts around a circle ahead of the fish."""
        cfg = self.config
        self.wander_theta += random.uniform(-cfg.wander_jitter, cfg.wander_jitter)
        ahead = Vector2(math.cos(self.body.heading), math.sin(self.body.heading))
        circle = self.position + ahead * cfg.wander_distance * self.scale
        offset = Vector2(math.cos(self.wander_theta), math.sin(self.wander_theta))
        self.acceleration += self._steer(circle + offset * cfg.wander_radius * self.scale)

    def flee(self, threat: Vector2, now: int) -> None:
        """Steer away from ``threat`` and start (or restart) a rush."""
        self.acceleration -= self._steer(threat)
        self.rush_step = 0
        self.next_rush_tick = now

    # -- motion -------------------------------------------------------------

    def update(self, now: int) -> None:
        self.velocity += self.acceleration
        limit(self.velocity, self.max_speed)
        self.position += self.velocity
        self.acceleration.update(0.0, 0.0)

        if self.velocity.length_squared() > 1e-6:
            self.body.heading = math.atan2(self.velocity.y, self.velocity.x)
        speed = self.velocity.length() / self.scale
        self.body.muscle_rate = speed * self.config.muscle_rate_per_speed
        self.body.move()
        self._update_rush(now)
        self.opacity = min(255, self.opacity + self.config.fade_in_per_frame)

    def _update_rush(self, now: int) -> None:
        """After a scare, swim fast and slow down step by step to cruising."""
        cfg = self.config
        if self.rush_step is None or now < self.next_rush_tick:
            return
        self.next_rush_tick = now + cfg.rush_step_ms
        if self.rush_step <= cfg.rush_steps:
            slowdown = cfg.rush_slowdown * self.scale * self.rush_step / cfg.rush_steps
            self.max_speed = self.rush_speed - slowdown
            self.rush_step += 1
        else:
            self.rush_step = None
            self.max_speed = self._cruise_speed()
            self.max_force = cfg.max_force * self.scale

    def wrap(self, width: int, height: int) -> None:
        """Reappear on the opposite side once fully off screen."""
        margin = self.skin_width
        if self.position.x < -margin:
            self.position.x = width
        elif self.position.x > width + margin:
            self.position.x = 0
        if self.position.y < -margin:
            self.position.y = height
        elif self.position.y > height + margin:
            self.position.y = 0

    def is_visible(self, width: int, height: int) -> bool:
        """``False`` while the fish is entirely off screen (nothing to draw)."""
        m = self.skin_width
        return -m <= self.position.x <= width + m and -m <= self.position.y <= height + m

    def draw(self, canvas: Canvas) -> None:
        canvas.draw_body(self.skin, self.body.segments(self.position), self.opacity)


# ---------------------------------------------------------------------------
# Scenery
# ---------------------------------------------------------------------------


def paint_pebble_bed(size: tuple[int, int], scale: float, rng: random.Random) -> pygame.Surface:
    """A dark bed of rounded pebbles, painted once at start-up."""
    w, h = size
    bed = pygame.Surface(size)
    bed.fill((3, 6, 12))
    count = int(w * h / (260 * scale * scale))
    for _ in range(count):
        rx = rng.uniform(6, 16) * scale
        ry = rx * rng.uniform(0.55, 0.85)
        x, y = rng.uniform(-rx, w + rx), rng.uniform(-ry, h + ry)
        shade = rng.uniform(0.0, 1.0)
        color = mix((9, 14, 26), (26, 36, 56), shade)
        stone = pygame.Surface((2 * rx + 2, 2 * ry + 2), pygame.SRCALPHA)
        pygame.draw.ellipse(stone, color, stone.get_rect().inflate(-2, -2))
        highlight = stone.get_rect().inflate(-rx * 0.9, -ry * 0.9).move(-rx * 0.2, -ry * 0.25)
        pygame.draw.ellipse(stone, (*mix(color, (70, 84, 110), 0.2), 70), highlight)
        bed.blit(pygame.transform.rotate(stone, rng.uniform(0, 180)), (x - rx, y - ry))
    return bed


def paint_inner_shadow(size: tuple[int, int], scale: float) -> pygame.Surface:
    """Dark rim around the pond, drawn above the fish like the original."""
    w, h = size
    shadow = pygame.Surface(size, pygame.SRCALPHA)
    depth = round(60 * scale)
    for i in range(depth):
        t = 1.0 - i / depth
        alpha = round(230 * t * t)
        pygame.draw.rect(shadow, (0, 0, 0, alpha), pygame.Rect(i, i, w - 2 * i, h - 2 * i), 1)
    return shadow


@dataclass
class Ripple:
    """An expanding ring on the water surface."""

    center: tuple[int, int]
    born: int

    def draw(self, canvas: Canvas, now: int, lifetime: int, max_radius: float) -> bool:
        """Draw the ring; returns ``False`` once it has faded out."""
        age = (now - self.born) / lifetime
        if age >= 1.0:
            return False
        radius = max_radius * age
        if radius > 1.0:
            canvas.draw_ripple(self.center, radius, round(120 * (1.0 - age)))
        return True


# ---------------------------------------------------------------------------
# The pond
# ---------------------------------------------------------------------------


@dataclass
class Touch:
    """Current finger (or mouse) state."""

    down: bool = False
    position: Vector2 = field(default_factory=Vector2)
    last_ripple: int = 0


class Pond:
    """Owns the canvas, the scenery, the koi and the main loop."""

    def __init__(self, config: PondConfig = CONFIG, seed: int | None = None) -> None:
        self.config = config
        self.rng = random.Random(seed)
        # Only video is needed; skipping audio avoids mixer issues on Android.
        pygame.display.init()
        self.canvas = open_canvas(config)
        self.clock = pygame.time.Clock()

        size = self.canvas.size
        self.scale = min(size) / config.reference_short_side
        self.canvas.set_scenery(
            paint_pebble_bed(size, self.scale, self.rng),
            paint_inner_shadow(size, self.scale),
        )
        self.skins = SkinLibrary(Path(__file__).with_name(config.skins_folder), config)

        self.school: list[Koi] = []
        self.ripples: list[Ripple] = []
        self.touch = Touch()
        self.next_birth = 0

    # -- population ---------------------------------------------------------

    def add_koi(self) -> None:
        """Add one koi; later fish swim higher, so they are drawn brighter."""
        cfg = self.config
        count = len(self.school)
        low, high = cfg.depth_brightness
        brightness = round(low + (high - low) * count / max(1, cfg.max_koi - 1))
        w, h = self.canvas.size
        margin = min(100 * self.scale, w / 4, h / 4)
        position = Vector2(self.rng.uniform(margin, w - margin), self.rng.uniform(margin, h - margin))
        surface, variety = self.skins.make(self.scale, self.rng)
        skin = self.canvas.add_skin(surface, cfg.body_segments, brightness)
        self.school.append(Koi(skin, surface.get_width(), variety, position, self.scale, cfg))

    # -- frame --------------------------------------------------------------

    def handle_events(self) -> bool:
        """Process input. Returns ``False`` when the app should quit."""
        back_key = getattr(pygame, "K_AC_BACK", pygame.K_ESCAPE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, back_key):
                return False
            # Touches arrive as mouse events on Android.
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.touch.down = True
                self.touch.position = Vector2(event.pos)
                self.touch.last_ripple = 0
            elif event.type == pygame.MOUSEMOTION and self.touch.down:
                self.touch.position = Vector2(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.touch.down = False
        return True

    def step(self, now: int) -> None:
        cfg = self.config
        if now >= self.next_birth and len(self.school) < cfg.max_koi:
            self.add_koi()
            self.next_birth = now + cfg.birth_interval_ms

        scare_radius_sq = (cfg.scare_radius * self.scale) ** 2
        w, h = self.canvas.size
        for koi in self.school:
            near = (
                self.touch.down
                and koi.position.distance_squared_to(self.touch.position) <= scare_radius_sq
            )
            if near:
                koi.flee(self.touch.position, now)
            else:
                koi.wander()
            koi.update(now)
            koi.wrap(w, h)

        if cfg.ripples and self.touch.down and now - self.touch.last_ripple >= cfg.ripple_interval_ms:
            pos = (round(self.touch.position.x), round(self.touch.position.y))
            self.ripples.append(Ripple(pos, now))
            self.touch.last_ripple = now

    def render(self, now: int) -> None:
        cfg = self.config
        canvas = self.canvas
        w, h = canvas.size
        canvas.begin_frame()
        for koi in self.school:
            if koi.is_visible(w, h):
                koi.draw(canvas)
        max_radius = cfg.ripple_max_radius * self.scale
        self.ripples = [
            r for r in self.ripples if r.draw(canvas, now, cfg.ripple_lifetime_ms, max_radius)
        ]
        canvas.end_frame()

    def run(self, max_frames: int | None = None) -> None:
        """Run the main loop; ``max_frames`` is for automated testing."""
        frame = 0
        try:
            while self.handle_events():
                now = pygame.time.get_ticks()
                self.step(now)
                self.render(now)
                self.clock.tick(self.config.fps)
                frame += 1
                if max_frames is not None and frame >= max_frames:
                    break
        finally:
            pygame.quit()


def main() -> None:
    Pond().run()


if __name__ == "__main__":
    main()
