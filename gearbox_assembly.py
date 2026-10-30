"""Jednoduchá třístupňová převodovka: skříň, hřídele, ložiska a ozubená kola.

Přehled převodů (modul 2 mm, osová vzdálenost 72 mm, součet zubů vždy 72):

    převod   vstup / výstup   i = z_out / z_in
      1.       18 / 54          3,0   (největší síla, nejnižší otáčky)
      2.       24 / 48          2,0
      3.       36 / 36          1,0   (přímý záběr)

Princip řazení: vstupní kola jsou pevně na vstupní hřídeli (x = -36),
výstupní kola tvoří posuvný blok na výstupní hřídeli (x = +36). Posunem
bloku podél osy Y zabírá vždy právě jedna dvojice. Rozmístění kol bylo
spočítáno tak, aby u ostatních dvojic, které by se radiálně překrývaly,
zůstala axiální mezera aspoň 7 mm.

Souřadnice: X = šířka, Y = délka (směr hřídelí), Z = výška; dělicí rovina
skříně a osy hřídelí leží v z = 0.

Pozn.: rovina xz má normálu -Y, proto kola, hřídele a ložiska stavím
symetricky kolem počátku a na místo je posouvám přes translate().

Nastavení na konci souboru:
  GEAR   1, 2 nebo 3 - který převod je zařazený
  COVER  "transparent", "solid" nebo "hidden" - horní polovina skříně
  EXPLODE  posun víka nahoru v mm (0 = zavřeno)
"""
import math

from cadbuildr.foundation import (
    Part,
    Assembly,
    Sketch,
    Point,
    Line,
    Polygon,
    Circle,
    Rectangle,
    Extrusion,
    show,
)

# --- Převodovka (mm) ---
SHAFT_DIST = 72.0                  # osová vzdálenost hřídelí
SHAFT_X = SHAFT_DIST / 2           # hřídele leží na x = +-36
GEAR_MAX_R = 56.0                  # největší kolo (54 zubů, modul 2)
CLEARANCE = 5.0                    # vůle mezi koly a stěnou

# --- Skříň ---
WALL = 5.0                         # tloušťka stěn a dna
END_WALL = 8.0                     # tloušťka čelních stěn (ložiska)
R_OUT = 12.0                       # vnější zaoblení hran
R_IN = R_OUT - WALL                # vnitřní zaoblení

INNER_W = 2 * (SHAFT_X + GEAR_MAX_R + CLEARANCE)   # 194 mm
INNER_H = GEAR_MAX_R + CLEARANCE                   # 61 mm od dělicí roviny
INNER_L = 90.0                                     # prostor pro 3 kola + posuv

HI = INNER_W / 2                   # polovina vnitřní šířky (97)
HX = HI + WALL                     # polovina vnější šířky (102)
H_OUT = INNER_H + WALL             # vnější výška poloviny (66)
L_OUT = INNER_L + 2 * END_WALL     # vnější délka (106)

FLANGE_W = 14.0                    # šířka příruby
FLANGE_T = 7.0                     # tloušťka příruby
HF = HX + FLANGE_W                 # polovina šířky včetně příruby (116)

# --- Ložiska a hřídele ---
BOSS_R = 27.0                      # poloměr náboje ložiska
BOSS_H = 6.0                       # přesah náboje za čelní stěnu
SEAT_R = 20.0                      # sedlo ložiska 6203 (vnější Ø 40)
SEAT_DEPTH = 12.0                  # hloubka sedla (šířka ložiska 12)
SHAFT_HOLE_R = 9.0                 # otvor pro hřídel Ø 17 + vůle

# --- Šrouby, výpusť, odvzdušnění ---
BOLT_R = 3.3                       # průchozí otvor pro M6
BOLT_Y = (-40.0, 0.0, 40.0)        # polohy šroubů podél délky
DRAIN_R = 5.0                      # výpustní otvor ve dně (spodní polovina)
DRAIN_Y = -30.0
BREATHER_R = 4.0                   # odvzdušňovací otvor ve víku (horní polovina)
BREATHER_Y = DRAIN_Y               # na stejné straně (y) jako výpusť


def _cyl_y(part, x, radius, y0, y1, cut=False):
    """Válec rovnoběžný s osou Y, osa v (x, z = 0), od světového y0 do y1 (y0 < y1)."""
    sketch = Sketch(part.xz())
    circle = Circle(Point(sketch, x, 0), radius)
    part.add_operation(Extrusion([circle], -y0, start=-y1, cut=cut))


def _cyl_z(part, x, y, radius, z0, z1, cut=True):
    """Válec rovnoběžný s osou Z, osa v (x, y), od z0 do z1 (z0 < z1)."""
    sketch = Sketch(part.xy())
    circle = Circle(Point(sketch, x, y), radius)
    part.add_operation(Extrusion([circle], z1, start=z0, cut=cut))


class HousingHalf(Part):
    """Jedna polovina skříně. upper=False: pod rovinou z = 0, upper=True: nad ní."""

    def __init__(self, upper=False):
        super().__init__()
        self.s = 1.0 if upper else -1.0   # smysl od dělicí roviny (+Z / -Z)
        self.upper = upper
        self._body()
        self._cavity()
        self._bearing_bosses()
        self._split_plane()
        self._bearing_seats()
        self._flange_bolts()
        if upper:
            self._breather()
        else:
            self._drain()

    def _body(self):
        """Vnější profil (miska s přírubami) vytažený symetricky podél Y."""
        s = self.s
        sketch = Sketch(self.xz())
        pen = sketch.pencil
        pen.move_to(0, s * H_OUT)
        pen.line_to(HX, s * H_OUT)
        pen.rounded_corner_then_line_to(HX, s * FLANGE_T, R_OUT)
        pen.line_to(HF, s * FLANGE_T)
        pen.line_to(HF, 0)
        pen.line_to(-HF, 0)
        pen.line_to(-HF, s * FLANGE_T)
        pen.line_to(-HX, s * FLANGE_T)
        pen.line_to(-HX, s * H_OUT)
        pen.rounded_corner_then_line_to(0, s * H_OUT, R_OUT)
        outer = pen.get_closed_shape()
        self.add_operation(Extrusion([outer], L_OUT / 2, start=-L_OUT / 2))

    def _cavity(self):
        """Dutina se zaoblenými hranami, mezi čelními stěnami."""
        s = self.s
        sketch = Sketch(self.xz())
        pen = sketch.pencil
        pen.move_to(0, s * INNER_H)
        pen.line_to(HI, s * INNER_H)
        pen.rounded_corner_then_line_to(HI, -s * 1.0, R_IN)
        pen.line_to(-HI, -s * 1.0)
        pen.line_to(-HI, s * INNER_H)
        pen.rounded_corner_then_line_to(0, s * INNER_H, R_IN)
        inner = pen.get_closed_shape()
        self.add_operation(
            Extrusion([inner], INNER_L / 2, start=-INNER_L / 2, cut=True)
        )

    def _bearing_bosses(self):
        """Čtyři náboje (2 hřídele x 2 čela), zatím jako plné válce."""
        y_in = L_OUT / 2
        y_out = y_in + BOSS_H
        for x in (-SHAFT_X, SHAFT_X):
            _cyl_y(self, x, BOSS_R, y_in, y_out)
            _cyl_y(self, x, BOSS_R, -y_out, -y_in)

    def _split_plane(self):
        """Odřízne vše na druhé straně dělicí roviny (z = 0)."""
        sketch = Sketch(self.xy())
        big = Rectangle.from_center_and_sides(Point(sketch, 0, 0), 400.0, 300.0)
        if self.upper:
            self.add_operation(Extrusion([big], 0.0, start=-60.0, cut=True))
        else:
            self.add_operation(Extrusion([big], 60.0, start=0.0, cut=True))

    def _bearing_seats(self):
        """Sedla ložisek (zahloubení zvenku) a průchozí otvory pro hřídele."""
        y_out = L_OUT / 2 + BOSS_H
        for x in (-SHAFT_X, SHAFT_X):
            _cyl_y(self, x, SEAT_R, y_out - SEAT_DEPTH, y_out + 1.0, cut=True)
            _cyl_y(self, x, SEAT_R, -y_out - 1.0, -y_out + SEAT_DEPTH, cut=True)
            _cyl_y(self, x, SHAFT_HOLE_R, -y_out - 1.0, y_out + 1.0, cut=True)

    def _flange_bolts(self):
        """Šest průchozích otvorů v přírubách (stejné polohy v obou polovinách)."""
        x_bolt = HX + FLANGE_W / 2
        z0, z1 = sorted((self.s * (FLANGE_T + 1.0), -self.s * 1.0))
        for x in (-x_bolt, x_bolt):
            for y in BOLT_Y:
                _cyl_z(self, x, y, BOLT_R, z0, z1)

    def _drain(self):
        """Výpustní otvor ve dně spodní poloviny."""
        _cyl_z(self, 0.0, DRAIN_Y, DRAIN_R, -H_OUT - 1.0, -INNER_H + 0.5)

    def _breather(self):
        """Odvzdušňovací otvor ve víku horní poloviny."""
        _cyl_z(self, 0.0, BREATHER_Y, BREATHER_R, INNER_H - 0.5, H_OUT + 1.0)


# =====================================================================
# Ozubená kola, hřídele, ložiska
# =====================================================================
GEAR_MODULE = 2.0            # mm
PRESSURE_ANGLE = 20.0        # stupně
GEAR_WIDTH = 10.0            # šířka kol (mm)
FLANK_PTS = 5                # body na jeden bok zubu (víc = přesnější, ale těžší model)

SHAFT_R = 8.5                # hřídel Ø 17 (vnitřní průměr ložiska 6203)
BEARING_W = 12.0             # šířka ložiska 6203
SHAFT_STUB = 20.0            # přesah hřídele za náboj ložiska

# Převody: (zubů vstup, zubů výstup)
RATIOS = [(18, 54), (24, 48), (36, 36)]

# Osové polohy kol (mm, světové y).
#   vstupní kola jsou pevně na hřídeli,
#   výstupní kola jsou v bloku: střed kola k = poloha bloku + OUT_OFFSET[k].
IN_Y = [0.0, -34.0, 34.0]            # 18, 24, 36 zubů
OUT_OFFSET = [17.0, 0.0, 34.0]       # 54, 48, 36 zubů (relativně v bloku)
BLOCK_POS = [17.0, 0.0, 34.0]        # poloha bloku pro převod 1, 2, 3
Y_SHIFT = -34.0                      # vycentrování celé skupiny v dutině


def _inv(a):
    return math.tan(a) - a


def gear_outline(teeth, module=GEAR_MODULE, n_flank=FLANK_PTS, phase=0.0):
    """Body obrysu evolventního kola (u, v), první zub je středěný na úhlu phase."""
    pitch_r = module * teeth / 2
    add_r = pitch_r + module
    ded_r = pitch_r - 1.25 * module
    base_r = pitch_r * math.cos(math.radians(PRESSURE_ANGLE))
    start_r = max(base_r, ded_r)
    offset = math.pi / (2 * teeth) + _inv(math.acos(base_r / pitch_r))

    radii = [start_r + (add_r - start_r) * i / (n_flank - 1) for i in range(n_flank)]
    half = [offset - _inv(math.acos(min(1.0, base_r / r))) for r in radii]
    flank = [(r, -h) for r, h in zip(radii, half)] + \
            [(r, +h) for r, h in reversed(list(zip(radii, half)))]

    pts = []
    for i in range(teeth):
        base = phase + 2 * math.pi * i / teeth
        tooth = [(r * math.cos(base + a), r * math.sin(base + a)) for r, a in flank]
        if base_r > ded_r:
            h0 = half[0]
            tooth.insert(0, (ded_r * math.cos(base - h0), ded_r * math.sin(base - h0)))
            tooth.append((ded_r * math.cos(base + h0), ded_r * math.sin(base + h0)))
        pts.extend(tooth)
    return pts


class SpurGear(Part):
    """Čelní kolo, osa rovnoběžná s Y, symetrické kolem y = 0, bez drážky pro pero."""

    def __init__(self, teeth, phase=0.0):
        super().__init__()
        half_w = GEAR_WIDTH / 2
        sketch = Sketch(self.xz())
        pts = [Point(sketch, round(u, 4), round(v, 4))
               for u, v in gear_outline(teeth, phase=phase)]
        lines = [Line(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
        self.add_operation(Extrusion([Polygon(lines)], half_w, start=-half_w))

        bore_sketch = Sketch(self.xz())
        bore = Circle(Point(bore_sketch, 0, 0), SHAFT_R)
        self.add_operation(
            Extrusion([bore], half_w + 1.0, start=-half_w - 1.0, cut=True)
        )


class Shaft(Part):
    """Hřídel Ø 17 mm, osa rovnoběžná s Y, symetrická kolem y = 0."""

    def __init__(self, length):
        super().__init__()
        sketch = Sketch(self.xz())
        circle = Circle(Point(sketch, 0, 0), SHAFT_R)
        self.add_operation(Extrusion([circle], length / 2, start=-length / 2))


class Bearing(Part):
    """Zjednodušené ložisko 6203 (Ø 17 / Ø 40 / šířka 12), symetrické kolem y = 0."""

    def __init__(self):
        super().__init__()
        half_w = BEARING_W / 2
        outer_sketch = Sketch(self.xz())
        outer = Circle(Point(outer_sketch, 0, 0), SEAT_R)
        self.add_operation(Extrusion([outer], half_w, start=-half_w))
        inner_sketch = Sketch(self.xz())
        inner = Circle(Point(inner_sketch, 0, 0), SHAFT_R)
        self.add_operation(
            Extrusion([inner], half_w + 1.0, start=-half_w - 1.0, cut=True)
        )


def _placed(part, x, y, color, transparency=0.0):
    part.translate([x, y, 0.0])
    if transparency:
        part.paint(color, transparency=transparency)
    else:
        part.paint(color)
    return part


def out_gear_y(gear):
    """Světové y výstupních kol (54, 48, 36 zubů) pro zařazený převod 1-3."""
    block = BLOCK_POS[gear - 1]
    return [block + off + Y_SHIFT for off in OUT_OFFSET]


def build_gearbox(gear=1, cover="transparent", explode=0.0):
    assert gear in (1, 2, 3), "gear musí být 1, 2 nebo 3"
    asm = Assembly()

    # Skříň
    lower = HousingHalf(upper=False)
    lower.paint("#8a8f98")
    asm.add_component(lower)
    if cover != "hidden":
        upper = HousingHalf(upper=True)
        if cover == "transparent":
            upper.paint("#6f9fd0", transparency=0.65)
        else:
            upper.paint("#6f9fd0")
        if explode:
            upper.translate_z(explode)
        asm.add_component(upper)

    y_end = L_OUT / 2 + BOSS_H                      # vnější čelo náboje (59)
    shaft_len = 2 * y_end + SHAFT_STUB              # 138 mm

    # Hřídele: vstupní (x = -36) vyčnívá na +Y, výstupní (x = +36) na -Y
    asm.add_component(_placed(Shaft(shaft_len), -SHAFT_X, SHAFT_STUB / 2, "#c3c7cf"))
    asm.add_component(_placed(Shaft(shaft_len), SHAFT_X, -SHAFT_STUB / 2, "#c3c7cf"))

    # Ložiska v sedlech (dosedají na dno sedla 12 mm od vnějšího čela)
    y_bear = y_end - BEARING_W / 2                  # 53
    for x in (-SHAFT_X, SHAFT_X):
        for y in (-y_bear, y_bear):
            asm.add_component(_placed(Bearing(), x, y, "#3a3d44"))

    # Vstupní kola (pevně na hřídeli)
    for (z_in, _), y in zip(RATIOS, IN_Y):
        asm.add_component(_placed(SpurGear(z_in), -SHAFT_X, y, "#e07b39"))

    # Výstupní kola (posuvný blok); fáze o půl rozteče, aby zub zapadl do mezery
    for (_, z_out), y in zip(RATIOS, out_gear_y(gear)):
        asm.add_component(
            _placed(SpurGear(z_out, phase=math.pi / z_out), SHAFT_X, y, "#3b82c4")
        )
    return asm


# --- Nastavení ---
GEAR = 1
COVER = "transparent"
EXPLODE = 0.0

show(build_gearbox(GEAR, COVER, EXPLODE))
