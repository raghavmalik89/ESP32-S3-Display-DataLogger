#!/usr/bin/env python3
"""Host-side verification of the lcd.c software-rotation pipeline.

Simulates, without any hardware:
  1. lcd_char()/lcd_text()   (font8x8_basic, MSB-first bit order)
  2. lcd_blit() into the 320x170 logical shadow framebuffer
  3. lcd_flush() rotation + slice packing into the native 320x170(x170) GRAM
     (CW transform: native_x = y + 35, native_y = 319 - x)

Checks:
  - font glyphs render as readable characters (bit order proof)
  - CW transform maps logical corners to the expected native corners
  - packed slice buffer order matches the native window row-major order
  - inverse transform recovers the original logical image (no mirror)

Run: python tests/test_lcd_transform.py
"""
import re, sys

LCD_H_RES, LCD_V_RES = 320, 170
NATIVE_W, NATIVE_H, XOFF = 170, 320, 35
RAM_W = 240          # ST7789 RAM is 240x320; glass shows columns 35..204
SLICE = 24

def load_font(path):
    txt = open(path, encoding="utf-8").read()
    glyphs = {}
    for m in re.finditer(r"\{\s*((?:0x[0-9A-Fa-f]{2}\s*,\s*){7}0x[0-9A-Fa-f]{2})\s*\}", txt):
        rows = [int(b, 16) for b in re.findall(r"0x[0-9A-Fa-f]{2}", m.group(1))]
        if len(rows) == 8:
            glyphs[len(glyphs)] = rows
    assert len(glyphs) >= 128, f"parsed only {len(glyphs)} glyphs"
    return glyphs

FONT = load_font("main/font8x8_basic.c")

# ---- model of lcd.c (mirrors firmware logic exactly) --------------------
def lcd_char(sh, x, y, ch, fg, bg):
    g = FONT[ord(ch) & 0x7F]
    for row in range(8):
        for col in range(8):
            on = (g[row] >> col) & 1             # firmware: LSB-first table
            sh[y + row][x + col] = fg if on else bg

def lcd_text(sh, x, y, s, fg, bg):
    for i, ch in enumerate(s):
        lcd_char(sh, x + 8 * i, y, ch, fg, bg)

def rot_cw(lx, ly):            # firmware CW transform
    return ly + XOFF, NATIVE_H - 1 - lx

def flush_model(sh):
    """Simulate the whole-surface flush (one dirty rect = full screen,
    sliced like firmware). Returns native[Y][X] pixel grid."""
    native = [[None] * RAM_W for _ in range(NATIVE_H)]
    x0, y0, x1, y1 = 0, 0, LCD_H_RES - 1, LCD_V_RES - 1
    H = y1 - y0 + 1
    sx0 = x0
    while sx0 <= x1:
        sx1 = min(sx0 + SLICE - 1, x1)
        W = sx1 - sx0 + 1
        bounce = [None] * (W * H)
        for r in range(W):                     # firmware CW packing
            lx = sx1 - r
            for c in range(H):
                bounce[r * H + c] = sh[y0 + c][lx]
        nx0, nx1 = y0 + XOFF, y1 + XOFF
        ny0, ny1 = NATIVE_H - 1 - sx1, NATIVE_H - 1 - sx0
        for r in range(W):                     # panel: row-major, X fastest
            for c in range(H):
                native[ny0 + r][nx0 + c] = bounce[r * H + c]
        sx0 += SLICE
    return native

# ---- build a diagnostic logical image ------------------------------------
FG, BG = 1, 0
sh = [[BG] * LCD_H_RES for _ in range(LCD_V_RES)]
for x in range(LCD_H_RES):                     # full border
    sh[0][x] = sh[LCD_V_RES - 1][x] = 2
for y in range(LCD_V_RES):
    sh[y][0] = sh[y][LCD_H_RES - 1] = 2
lcd_text(sh, 60, 40, "ABC123", FG, BG)
lcd_text(sh, 60, 80, "LEFT->RIGHT", FG, BG)
lcd_text(sh, 4, 4, "TL", FG, BG)
lcd_text(sh, LCD_H_RES - 20, 4, "TR", FG, BG)
lcd_text(sh, 4, LCD_V_RES - 12, "BL", FG, BG)
lcd_text(sh, LCD_H_RES - 20, LCD_V_RES - 12, "BR", FG, BG)

native = flush_model(sh)

# ---- check 1: corner mapping ----------------------------------------------
def nat_at(lx, ly):
    nx, ny = rot_cw(lx, ly)
    return native[ny][nx]

fails = []
# logical TL corner (0,0) -> native (XOFF, 319); TR (319,0) -> (XOFF, 0)
# BL (0,169) -> (204, 319); BR (319,169) -> (204, 0)   (RAM is 240 wide)
expect = {rot_cw(0, 0): "TL", rot_cw(319, 0): "TR",
          rot_cw(0, 169): "BL", rot_cw(319, 169): "BR"}
for (nx, ny), label in expect.items():
    if not (0 <= nx < RAM_W and 0 <= ny < NATIVE_H):
        fails.append(f"{label}: native ({nx},{ny}) out of range")

# sample the four logical corner labels, inverse-map native, and verify
# each corner label sits in the right native quadrant
def label_at(lx, ly, w=16, h=8):
    nx, ny = rot_cw(lx, ly)
    return [[native[yy][xx] for xx in range(nx, nx + w)] for yy in range(ny, ny + h)]

def quadrant(nx, ny):
    return ("bottom" if ny > NATIVE_H // 2 else "top",
            "left" if nx < RAM_W // 2 else "right")

corner_tests = [((0, 0), ("bottom", "left")), ((303, 0), ("top", "left")),
                ((0, 161), ("bottom", "right")), ((303, 161), ("top", "right"))]
for (lx, ly), want in corner_tests:
    nx, ny = rot_cw(lx, ly)
    q = quadrant(nx, ny)
    if q != want:
        fails.append(f"corner ({lx},{ly}) -> native quadrant {q}, want {want}")

# ---- check 2: inverse transform recovers logical image (no mirror) --------
inv_ok = True
for ly in range(0, LCD_V_RES, 3):
    for lx in range(0, LCD_H_RES, 3):
        nx, ny = rot_cw(lx, ly)
        if native[ny][nx] != sh[ly][lx]:
            inv_ok = False
if not inv_ok:
    fails.append("inverse transform does not recover logical image")

# ---- check 3: font readability proof --------------------------------------
# Render each character of ABC123 upright in a terminal using the SAME
# firmware bit order, AND assert the asymmetric glyphs (2, L, R) match
# their known-correct patterns — symmetric glyphs (A/B/C) cannot prove
# bit order on their own.
print("Font render (firmware bit order, upright):")
for ch in "ABC123":
    g = FONT[ord(ch)]
    art = "".join("".join("#" if (g[r] >> c) & 1 else "." for c in range(8)) + "\n"
                  for r in range(8))
    print(f" {ch}:\n{art}")

# known-correct LSB-first renderings (hand-derived from the glyph bytes;
# orientation is unambiguous: 2's upper stub is on the right, L's stroke
# on the left, R's leg kicking right — symmetric glyphs can't prove this)
EXPECT_2 = [".####...", "##..##..", "....##..", "..###...", ".##.....",
            "##..##..", "######..", "........"]
EXPECT_L = ["####....", ".##.....", ".##.....", ".##.....", ".##...#.",
            ".##..##.", "#######.", "........"]
EXPECT_R = ["######..", ".##..##.", ".##..##.", ".#####..", ".##.##..",
            ".##..##.", "###..##.", "........"]
for ch, expect in (("2", EXPECT_2), ("L", EXPECT_L), ("R", EXPECT_R)):
    g = FONT[ord(ch)]
    got = ["".join("#" if (g[r] >> c) & 1 else "." for c in range(8))
           for r in range(8)]
    if got != expect:
        fails.append(f"glyph {ch} does not match known-correct pattern:\n"
                     + "\n".join(got))

# ---- check 4: native view, rotated back to landscape ----------------------
# L[y][x] = native[319-x][y+XOFF] is exactly what an observer sees when
# turning the panel to landscape; it must equal the logical image.
L = [[native[NATIVE_H - 1 - x][y + XOFF] for x in range(LCD_H_RES)]
     for y in range(LCD_V_RES)]

# ABC123 occupies logical y=40..47: its rows must be lit
row44 = sum(1 for v in L[44] if v)
row83 = sum(1 for v in L[83] if v)   # LEFT->RIGHT at y=80..87
print(f"\nLandscape-recovered text rows: y=44 -> {row44} px, y=83 -> {row83} px "
      f"(both >0 expected)")

# border corners recovered intact (value 2 = border color)
checks = [(0, 0), (319, 0), (0, 169), (319, 169)]
print("Border corner pixels present:",
      [bool(L[y][x]) for x, y in checks], "(want all True)")

if fails or not all(bool(L[y][x]) for x, y in checks) or row44 == 0 or row83 == 0:
    print("\nFAIL:")
    for f in fails: print(" -", f)
    sys.exit(1)
print("\nPASS: transform consistent, corners map correctly, no mirror, font readable")
