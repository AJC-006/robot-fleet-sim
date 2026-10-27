"""keep: the dragon's citadel estate, ~340 x 210 m.

Layout (metres, +x east, +y north; lake surface z = -0.8, everything else walkable at z = 0):
  forecourt + outer road   x[-141,19]  y[-108.5,-91]    walled plaza in front of the gatehouse, joins the promenade
  gatehouse + main gate    x[-129,-71] y[-91,-64]       passage 22 m wide x 26 m tall, guard rooms 1.6 m doors
  courtyard (open sky)     x[-146,-56] y[-64,20]        90 x 84 m garden courtyard, dragon lane x[-111,-89] then east y[-41,-19]
  great hall               x[-52,8]    y[-64,58]        60 x 122 x 40 m, dragon statue, colonnade, burgundy carpet
  dragon's hoard           x[-52,8]    y[58,86]         cave behind the hall (22 x 26 m passage)
  north ward               x[-146,-56] y[24,86]         human scale: armoury, forge, kitchen, chapel, crypt, towers, servants' rooms
  promenade                x[13,19]    y[-91,68]        along the hall's east wall (two 2 m posterns), ring path around the lake
  lake                     x[21,163]   y[-50,50]        8-20 m deep, stone quay with a 0.7 m parapet, drowned dragon temple
Ground robots: quay parapet and 30 m walls keep everything in; the map covers only what is reachable.
Dragon route (>= 20 m wide, >= 24 m tall): gate -> courtyard lane -> hall passage -> hall lane -> hoard passage.
"""
import math
import random

import dragon_statue
import keep_lake as KL
from common import banner, barrel, crate, hwall, torch, vwall, zfloor
from keep_props import *  # noqa: F401,F403
import keep_props as KP

NAME = "keep"
BOUNDS = (-165.0, -112.0, 185.0, 100.0)
OUTER = (-165.5, -112.5, 185.5, 100.5)
CHUNK = 40.0
CHUNK_ORIGIN = (-165.0, -112.0)
EXPORT = "GLTF"
MAP_RES = 0.10
MAP_CROP = True
SPAWN = (-100.0, -95.0, math.pi / 2)

SPAWNS = {
    "gate": (-100.0, -80.0, math.pi / 2),
    "hall": (-39.0, -5.0, math.pi / 2),
    "courtyard": (-100.0, -45.0, math.pi / 2),
    "crypt": (-120.3, 60.0, math.pi / 2),
    "quay": (16.0, -20.0, math.pi / 2),
    "hoard": (-39.0, 66.0, math.pi / 2),
    "forecourt": (-100.0, -95.0, math.pi / 2),
    "armoury": (-134.0, 35.0, 0.0),
    "bank_east": (173.0, 0.0, math.pi),
}
GOALS = [
    ("forecourt_gate", -100.0, -95.0, math.pi / 2, "forecourt"),
    ("gatehouse_w", -121.0, -84.0, 0.0, "gate"),
    ("court_fountain", -83.0, 8.5, 0.0, "courtyard"),
    ("court_beds", -135.0, -10.0, math.pi / 2, "courtyard"),
    ("hall_carpet", -39.0, 0.0, math.pi / 2, "hall"),
    ("hall_statue", -8.0, -22.0, math.pi / 2, "hall"),
    ("hoard_gold", -17.0, 64.0, math.pi, "hoard"),
    ("armoury", -134.0, 35.0, 0.0, "ward"),
    ("crypt_ring", -120.3, 68.0, math.pi / 2, "ward"),
    ("chapel", -66.0, 35.0, math.pi, "ward"),
    ("quay_north", 16.0, 50.0, math.pi / 2, "grounds"),
    ("bank_east", 173.0, 0.0, math.pi, "grounds"),
]
SHORT_PAIRS = []
for _i, _a in enumerate(GOALS):
    for _b in GOALS[_i + 1:]:
        _d = math.hypot(_a[1] - _b[1], _a[2] - _b[2])
        if _d < 60.0:
            SHORT_PAIRS.append([_a[0], _b[0], round(_d, 1)])

DRONE = {
    "takeoff": (-100.0, -28.0),
    "cruise": 14.0,
    "waypoints": [
        ("court_up", -100.0, -28.0, 14.0),
        ("court_west", -128.0, -12.0, 16.0),
        ("hall_entry", -44.0, -30.0, 14.0),
        ("hall_mid", -39.0, 10.0, 22.0),
        ("hall_far", -39.0, 44.0, 26.0),
        ("lake_a", 60.0, 0.0, 12.0),
        ("lake_b", 120.0, -30.0, 14.0),
        ("lake_c", 150.0, 22.0, 10.0),
    ],
}


def _auv():
    def wp(name, x, y, above):
        z = max(-15.0, min(-2.5, KL.bed(x, y) + above))
        return (name, x, y, round(z, 1))
    return {
        "water_level": KL.WATER_Z,
        "spawn": (24.0, 0.0, -2.0, 0.0),
        "waypoints": [
            wp("slipway_exit", 40.0, 0.0, 5.0),
            wp("approach", 54.0, 6.0, 6.0),
            wp("temple_gate", 68.0, 0.0, 6.5),
            wp("nave_mid", 90.0, 0.0, 6.5),
            wp("head_view", 98.0, -3.0, 6.5),
            wp("south_aisle", 82.0, -14.0, 5.0),
            wp("apse", 114.0, 0.0, 6.5),
            wp("wreck_side", 52.0, 14.0, 6.0),
        ],
    }


AUV = _auv()

# translucent water surface (visual only: no collision, the AUV floats through graded buoyancy). Seen from above it is a
# dark violet-teal mirror, from below a glowing ceiling.
WATER_SDF = (
    '<model name="lake_surface"><static>true</static><pose>%.2f %.2f %.2f 0 0 0</pose><link name="link">'
    '<visual name="surface"><cast_shadows>false</cast_shadows><transparency>%.2f</transparency>'
    '<geometry><box><size>%.1f %.1f 0.02</size></box></geometry>'
    '<material><ambient>0.05 0.16 0.24 1</ambient><diffuse>0.05 0.20 0.28 1</diffuse><specular>0.7 0.6 1.0 1</specular>'
    '<emissive>0.02 0.07 0.10 1</emissive>'
    '<pbr><metal><metalness>0.3</metalness><roughness>0.08</roughness></metal></pbr></material></visual>'
    '</link></model>' % ((KL.WX0 + KL.WX1) / 2, (KL.WY0 + KL.WY1) / 2, KL.WATER_Z, 0.45, KL.WX1 - KL.WX0 + 1.0, KL.WY1 - KL.WY0 + 1.0))

SCENE = {
    # 818 static collision prims cost ~0.25 s of CPU per sim second at 0.002 s steps (RTF 0.7 idle); 0.004 gives RTF 1.0.
    "max_step_size": "0.004",
    # Engine, solver and detector come from sdfgen.PHYSICS (DART, dantzig, ode: docs/notes/ground.md G2; the
    # dartsim default detector is ode, not FCL as noted earlier). Earlier worlds-lane notes:
    # 818 static collision prims: the default detector costs ~0.7 core idle, the bullet detector ~0.26, but
    # with bullet the round robot glitched in 2 of 2 Nav2 runs (stuck in a hall bay, then flung 3 m near hall_statue);
    # the default passed the same runs. Keep it.
    # "collision_detector": "bullet",
    # "engine": "bullet-featherstone",  # breaks pivot turns: default spinning friction (docs/notes/ground.md G2)
    "ambient": (0.20, 0.14, 0.28),
    "background": (0.03, 0.015, 0.07),
    "shadows": False,
    "sun": {"color": (0.24, 0.2, 0.46), "dir": (0.3, 0.35, -1.0), "shadows": False},
    # lake physics for the knucker AUV: graded buoyancy (fresh water below the surface, air above), enabled for
    # the AUV model only so no ground robot is ever touched, and the DVL sensor system
    "plugins": [
        '<plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">'
        '<graded_buoyancy><default_density>999.103</default_density>'
        '<density_change><above_depth>-0.8</above_depth><density>1.225</density></density_change>'
        '</graded_buoyancy><enable>knucker</enable></plugin>',
        '<plugin filename="gz-sim-dvl-system" name="gz::sim::systems::DopplerVelocityLogSystem"/>',
        WATER_SDF,
    ],
}
PREVIEW = {
    "cam": (10.0, -300.0, 320.0), "target": (10.0, -5.0, 0.0), "lens": 26,
    "world": (0.05, 0.025, 0.1), "world_strength": 0.5,
    "hide_mats": ["roof", "water", "sky_h", "sky_l", "sky_m", "sky_t", "star", "moon", "moon_halo", "ridge"],
    "extra_lights": [("SUN", (0, 0, 10), (0.55, 0.45, 1.0), 0.7, 0),
                     ("POINT", (-40, -30, 120), (0.8, 0.45, 1.0), 6.0e4, 5.0)],
    "light_rot": (math.radians(40), math.radians(15), math.radians(35)),
    "light_gain": 4.0,
    "roof_off": ["lake", "temple", "head", "gate", "court"],
    "show_in": {"gate": ["sky_h", "sky_l", "sky_m", "sky_t", "star", "moon", "moon_halo", "ridge"],
                "court": ["sky_h", "sky_l", "sky_m", "sky_t", "star", "moon", "moon_halo", "ridge"],
                "promenade": ["sky_h", "sky_l", "sky_m", "sky_t", "star", "moon", "moon_halo", "ridge"],
                "lake": ["water", "sky_h", "sky_l", "sky_m", "sky_t", "star", "moon", "moon_halo", "ridge"]},
    "views": [
        ("gate", (-100.0, -135.0, 14.0), (-100.0, -70.0, 14.0), 22),
        ("court", (-140.0, -58.0, 14.0), (-95.0, -10.0, 3.0), 22),
        ("hall", (-46.0, -58.0, 5.0), (-30.0, 20.0, 14.0), 20),
        ("statue", (-35.0, -12.0, 4.0), (-8.0, 14.0, 12.0), 22),
        ("statue2", (-42.0, -44.0, 12.0), (-8.0, 17.0, 11.0), 30),
        ("hoard", (-45.0, 62.0, 3.0), (-18.0, 76.0, 3.0), 22),
        ("ward", (-128.0, 27.0, 2.2), (-96.0, 40.0, 1.5), 20),
        ("crypt", (-120.0, 52.0, 2.0), (-110.0, 68.0, 1.5), 20),
        ("lake", (21.0, -60.0, 60.0), (95.0, 0.0, -10.0), 24),
        ("temple", (60.0, -8.0, -9.0), (100.0, -16.0, -14.0), 24),
        ("head", (84.0, -24.0, -12.0), (99.0, -18.0, -16.0), 26),
        ("promenade", (16.0, -85.0, 3.0), (16.0, -20.0, 3.0), 24),
    ],
}

MATS = {
    "wall": dict(tex="castle_brick_07", tile=2.0, tint=(0.36, 0.3, 0.52)),
    "wallg": dict(tex="castle_brick_07", tile=5.0, tint=(0.36, 0.3, 0.52)),
    "wallg_out": dict(tex="castle_brick_07", tile=5.0, tint=(0.28, 0.24, 0.4)),
    "roof": dict(tex="castle_brick_07", tile=5.0, tint=(0.26, 0.22, 0.36)),
    "floor": dict(tex="stone_tiles_02", tile=2.0, tint=(0.42, 0.4, 0.5)),
    "floor_big": dict(tex="stone_tiles_02", tile=6.0, tint=(0.34, 0.31, 0.42)),
    "cobble_big": dict(tex="cobblestone_floor_04", tile=4.0, tint=(0.5, 0.46, 0.6)),
    "paving": dict(tex="stone_tiles_02", tile=3.0, tint=(0.4, 0.38, 0.48)),
    "rock_floor": dict(tex="rock_face_03", tile=4.0, tint=(0.36, 0.3, 0.38)),
    "ground": dict(tex="mossy_cobblestone", tile=4.0, tint=(0.28, 0.34, 0.3)),
    "soil": dict(tex="rock_face_03", tile=1.5, tint=(0.3, 0.22, 0.2)),
    "moss": dict(tex="mossy_cobblestone", tile=1.5, tint=(0.45, 0.55, 0.45)),
    "rock": dict(tex="rock_face_03", tile=2.0, tint=(0.45, 0.4, 0.5)),
    "pillar": dict(tex="rustic_stone_wall_02", tile=3.0, tint=(0.42, 0.38, 0.55)),
    "stone": dict(tex="rustic_stone_wall_02", tile=1.2, tint=(0.38, 0.35, 0.46)),
    "planter": dict(tex="rustic_stone_wall_02", tile=1.2, tint=(0.34, 0.32, 0.4)),
    "quay": dict(tex="concrete_layers_02", tile=3.0, tint=(0.36, 0.34, 0.44)),
    "wood": dict(tex="dark_wood", tile=1.0, tint=(0.8, 0.7, 0.8)),
    "gold": dict(color=(0.9, 0.6, 0.06), metal=1.0, rough=0.3),
    "gold_old": dict(color=(0.5, 0.3, 0.03), metal=1.0, rough=0.45),
    "gem": dict(color=(0.55, 0.05, 0.4), metal=0.0, rough=0.1, emit=((0.6, 0.1, 0.5), 0.6)),
    "bone": dict(color=(0.45, 0.4, 0.32), rough=0.8),
    "dragon_bone": dict(tex="rock_face_03", tile=3.0, tint=(0.7, 0.62, 0.5)),
    "black_socket": dict(color=(0.005, 0.005, 0.01), rough=1.0),
    "dragon": dict(color=(0.13, 0.11, 0.06), metal=0.85, rough=0.5),
    "wing": dict(color=(0.07, 0.05, 0.09), metal=0.2, rough=0.6),
    "eye": dict(color=(0.8, 0.1, 0.5), emit=((0.9, 0.12, 0.6), 3.0)),
    "flame": dict(color=(1.0, 0.45, 0.1), emit=((1.0, 0.45, 0.1), 5.0)),
    "flame_m": dict(color=(0.7, 0.1, 0.9), emit=((0.7, 0.15, 1.0), 5.0)),
    "rune": dict(color=(0.5, 0.15, 0.9), emit=((0.5, 0.15, 1.0), 3.5)),
    "lamp_g": dict(color=(1.0, 0.7, 0.35), emit=((1.0, 0.6, 0.25), 3.0)),
    "banner_p": dict(color=(0.1, 0.015, 0.18), rough=0.9),
    "banner_c": dict(color=(0.22, 0.015, 0.04), rough=0.9),
    "trim": dict(color=(0.55, 0.38, 0.08), metal=0.8, rough=0.4),
    "iron": dict(color=(0.03, 0.03, 0.04), metal=0.9, rough=0.5),
    "carpet": dict(tex="dirty_carpet", tile=1.5, tint=(0.30, 0.02, 0.06)),
    "rubble": dict(color=(0.07, 0.065, 0.08), rough=1.0),
    "steel": dict(color=(0.3, 0.3, 0.34), metal=0.9, rough=0.35),
    "leather": dict(color=(0.1, 0.045, 0.03), rough=0.8),
    "hay": dict(color=(0.45, 0.33, 0.1), rough=1.0),
    "cloth": dict(color=(0.3, 0.28, 0.24), rough=1.0),
    "glass": dict(color=(0.3, 0.2, 0.7), emit=((0.35, 0.2, 0.9), 1.2), rough=0.1),
    "hedge": dict(color=(0.03, 0.09, 0.06), rough=1.0),
    "bloom": dict(color=(0.7, 0.1, 0.5), emit=((0.6, 0.1, 0.45), 0.5), rough=0.8),
    "leaf": dict(color=(0.04, 0.1, 0.08), rough=1.0),
    "marble": dict(color=(0.55, 0.5, 0.6), rough=0.35),
    "water_p": dict(color=(0.12, 0.1, 0.3), rough=0.05, metal=0.3, emit=((0.15, 0.12, 0.5), 0.5)),
    "water": dict(color=(0.02, 0.05, 0.16), rough=0.05, metal=0.2, alpha=0.55),
    "mud": dict(tex="rock_face_03", tile=5.0, tint=(0.34, 0.38, 0.44), color=(0.15, 0.17, 0.2)),
    "temple": dict(tex="rustic_stone_wall_02", tile=2.5, tint=(0.3, 0.38, 0.38)),
    "weed": dict(color=(0.05, 0.22, 0.12), rough=0.9),
    "wreck": dict(tex="dark_wood", tile=1.2, tint=(0.5, 0.55, 0.5)),
    "sky_h": dict(color=(0.3, 0.06, 0.28), emit=((0.5, 0.1, 0.4), 1.0)),
    "sky_l": dict(color=(0.14, 0.04, 0.3), emit=((0.2, 0.06, 0.45), 1.0)),
    "sky_m": dict(color=(0.06, 0.03, 0.2), emit=((0.09, 0.04, 0.28), 1.0)),
    "sky_t": dict(color=(0.02, 0.015, 0.08), emit=((0.03, 0.02, 0.12), 1.0)),
    "star": dict(color=(1, 1, 1), emit=((0.9, 0.85, 1.0), 6.0)),
    "moon": dict(color=(0.8, 0.75, 1.0), emit=((0.75, 0.65, 1.0), 2.5)),
    "moon_halo": dict(color=(0.4, 0.2, 0.7), emit=((0.45, 0.25, 0.85), 0.8), alpha=0.35),
    "ridge": dict(color=(0.015, 0.01, 0.03), rough=1.0),
}

rng = random.Random(7)
HC, HH = 30.0, 40.0          # curtain wall / great hall wall height
GW = "wallg"
ROUTE = [(-100.0, -75.0), (-100.0, -30.0), (-39.0, -30.0), (-39.0, 75.0)]    # dragon route polyline
ROUTE_HALF = 10.0   # dragon needs 20 m clear (+/-10 m around the polyline)


def rect(W, mat, x0, y0, x1, y1, z0=-0.1, h=0.1):
    W.boxp(mat, x0, y0, x1, y1, z0, h, solid=False)


def floors(W):
    for (x0, y0, x1, y1) in [(-165, -112, 19, 100), (19, -112, 185, -52), (19, 52, 185, 100), (165, -52, 185, 52)]:
        W.col_box((x0 + x1) / 2, (y0 + y1) / 2, -0.2, x1 - x0, y1 - y0, 0.2)
        rect(W, "ground", x0, y0, x1, y1, -0.14, 0.1)
    rect(W, "cobble_big", -146, -64, -56, 20)          # courtyard
    rect(W, "floor_big", -52, -64, 8, 58)              # hall
    rect(W, "rock_floor", -52, 58, 8, 86)              # hoard
    rect(W, "cobble_big", -115, -91, -85, -64)         # gate passage
    rect(W, "floor", -129, -91, -115, -64)
    rect(W, "floor", -85, -91, -71, -64)
    rect(W, "cobble_big", -141, -108.5, 19.5, -91)     # forecourt + road
    rect(W, "paving", 13, -91, 19, 68)                 # promenade
    rect(W, "paving", 19, -68, 182, -52)
    rect(W, "paving", 19, 52, 182, 68)
    rect(W, "paving", 165, -52, 182, 52)
    rect(W, "floor", -146, 24, -56, 86)                # north ward
    rect(W, "floor_big", -56, -41, -52, -19)           # hall passage threshold
    rect(W, "floor_big", -50, 56, -28, 60)
    rect(W, "floor_big", -111, -70, -89, -64)
    rect(W, "floor", -146, 24, -122, 46)
    # dark stone in the crypt and cobbles in the forge/kitchen
    zfloor(W, "rubble", -122, 50, -98, 86, h=0.012)
    zfloor(W, "cobble_big", -122, 24, -76, 46, h=0.012)


def tall_walls(W):
    # south curtain with the main gate, west curtain, north curtain
    hwall(W, -67, -149, -54, HC, GW, [(-100, 22, 26)], t=6)
    hwall(W, -67, -54, 10, HH, GW, [], t=6)
    vwall(W, -149, -67, 89, HC, GW, [], t=6)
    hwall(W, 89, -149, 10, HC, GW, [], t=6)
    # great hall: west wall (dragon passage 22 x 26, ward door), east wall (two 2 m posterns), hoard divider
    vwall(W, -54, -67, 58, HH, GW, [(-30, 22, 26), (48, 1.6, 2.4)], t=4)
    vwall(W, -54, 58, 89, HC, GW, [(76, 1.4, 2.3)], t=4)
    vwall(W, 10, -67, 58, HH, GW, [(-40, 2.0, 2.6), (40, 2.0, 2.6)], t=4)
    vwall(W, 10, 58, 89, HC, GW, [], t=4)
    hwall(W, 58, -54, 10, HH, GW, [(-39, 22, 26)], t=4)
    hwall(W, 22, -149, -54, HC, GW, [(-110, 2.4, 2.8), (-66, 2.4, 2.8)], t=4)
    # gatehouse: passage walls, outer walls, front wall with the gate
    vwall(W, -113, -90, -70, 36.0, GW, [(-84.5, 1.6, 2.3), (-75, 1.6, 2.3)], t=4)
    vwall(W, -87, -90, -70, 36.0, GW, [(-84.5, 1.6, 2.3), (-75, 1.6, 2.3)], t=4)
    vwall(W, -128, -90, -70, 36.0, GW, [], t=2)
    vwall(W, -72, -90, -70, 36.0, GW, [], t=2)
    hwall(W, -90, -128, -72, 36.0, GW, [(-100, 22, 26)], t=2)
    hwall(W, -79.6, -128, -115, 5.9, "wall", [(-121, 1.2, 2.2)], t=0.4)
    hwall(W, -79.6, -85, -72, 5.9, "wall", [(-79, 1.2, 2.2)], t=0.4)
    # roofs: guard room ceilings and the gatehouse tops
    for (x0, x1) in [(-129, -111), (-89, -71)]:
        W.boxp("roof", x0, -91, x1, -69, 6.0, 0.4, tag="roof")
        W.boxp("roof", x0, -91, x1, -64, 36.0, 1.0, tag="roof")
    W.boxp("roof", -146, 24, -56, 86, 6.0, 0.4, tag="roof")
    W.boxp("roof", -52, -64, 8, 58, HH, 1.5, tag="roof")
    W.boxp("roof", -52, 58, 8, 86, HC, 1.5, tag="roof")
    # towers, merlons
    for (x, y, r, h) in [(-149, -67, 10, 44), (-149, 89, 10, 44), (10, 89, 9, 50), (-128, -91, 4.5, 46), (-72, -91, 4.5, 46),
                         (-152, -40, 7, 38), (-152, 0, 7, 38), (-135, -70, 7, 38), (-65, -70, 7, 38), (-134, 93.5, 6, 38), (-66, 93.5, 6, 38)]:
        KP.tower(W, x, y, r, h)
    for (x0, y0, x1, y1, t, z) in [(-149, -67, -54, -67, 6, HC), (-149, -67, -149, 89, 6, HC), (-149, 89, 10, 89, 6, HC),
                                   (-54, -67, 10, -67, 6, HH), (-54, -67, -54, 58, 4, HH), (10, -67, 10, 58, 4, HH),
                                   (-149, 22, -54, 22, 4, HC), (-128, -90, -72, -90, 2, 36.0)]:
        KP.merlons(W, x0, y0, x1, y1, z, t)
    # string courses on the courtyard faces and interior hall faces
    for z in (10.0, 20.0):
        W.boxp("stone", -146.4, -64, -145.9, 20, z, 0.7, solid=False)
        W.boxp("stone", -146, -63.9, -56, -63.4, z, 0.7, solid=False)
        W.boxp("stone", -56.5, -64, -56, -41, z, 0.7, solid=False)
        W.boxp("stone", -56.5, -19, -56, 20, z, 0.7, solid=False)
    for z in (14.0, 28.0):
        W.boxp("stone", -52, -63.6, -51.5, 58, z, 0.8, solid=False)
        W.boxp("stone", 7.5, -63.6, 8, 58, z, 0.8, solid=False)
    # ward partitions (1.1 - 2 m doors)
    P = "wall"
    hwall(W, 46, -146, -56, 5.9, P, [(-134, 1.4), (-110, 1.6), (-87, 1.4), (-66, 2.0)])
    hwall(W, 50, -146, -56, 5.9, P, [(-134, 1.4), (-110, 1.6), (-87, 1.1), (-66, 1.4)])
    vwall(W, -122, 24, 46, 5.9, P, [(35, 1.6)])
    vwall(W, -98, 24, 46, 5.9, P, [(30, 1.4)])
    vwall(W, -76, 24, 46, 5.9, P, [(35, 1.6)])
    vwall(W, -122, 50, 86, 5.9, P, [(78, 1.4)])
    vwall(W, -98, 50, 86, 5.9, P, [(76, 1.4), (58, 1.1)])
    vwall(W, -76, 50, 86, 5.9, P, [(58, 1.2), (76, 1.4)])
    hwall(W, 66, -146, -122, 5.9, P, [(-134, 1.4)])
    hwall(W, 66, -98, -76, 5.9, P, [(-87, 1.1)])
    hwall(W, 66, -76, -56, 5.9, P, [(-66, 1.6)])
    for x in range(-144, -56, 4):                      # ward ceiling beams
        W.boxp("wood", x - 0.2, 24, x + 0.2, 86, 5.5, 0.5, solid=False, tag="roof")
    # parapet walls: forecourt, road, promenade, banks (2.5 m, closed)
    S = "wallg_out"
    hwall(W, -109.25, -141.75, 19.5, 2.5, S, [], t=1.5)
    vwall(W, -141.75, -109.25, -90.25, 2.5, S, [], t=1.5)
    hwall(W, -90.25, -141.75, -129, 2.5, S, [], t=1.5)
    hwall(W, -90.25, -71, 12.5, 2.5, S, [], t=1.5)
    vwall(W, 12.5, -91, -69, 2.5, S, [], t=1.0)
    vwall(W, 19.5, -109, -69, 2.5, S, [], t=1.0)
    hwall(W, -68.5, 19, 182.5, 2.5, S, [], t=1.0)
    hwall(W, 68.5, 12.5, 182.5, 2.5, S, [], t=1.0)
    vwall(W, 182.5, -68.5, 68.5, 2.5, S, [], t=1.0)


def hall(W):
    # colonnade east of the lane, flush pilasters + windows + banners on the walls, braziers, chandeliers
    ys = list(range(-58, 56, 12))
    for i, y in enumerate(ys):
        KP.pillar_big(W, -27.5, y, 37.0)    # edge at x=-29: flush with the 20 m dragon corridor (lidar landmark for the hall entrance)
        if y != ys[-1]:
            if i % 2 == 0:       # closed bay: hedge planter flush against the lane edge (lidar landmark), open bays keep 2.8 m gaps
                KP.parterre(W, -29.0, y + 1.5, -27.6, y + 10.5)
            else:
                for (a, b) in [(1.8, 4.7), (7.3, 10.2)]:
                    W.boxp("stone", -29.0, y + a, -25.8, y + b, 0.0, 0.75)
                    W.boxp("trim", -29.0, y + a - 0.1, -25.7, y + b + 0.1, 0.75, 0.05, solid=False)
    for y in [-60, -52, -46, -14, -6, 2, 10, 18, 26, 34, 42, 54]:
        W.boxp("pillar", -52, y - 1.5, -50.5, y + 1.5, 0.0, 28.0)
        W.boxp("stone", -52, y - 1.8, -50.3, y + 1.8, 28.0, 0.6, solid=False)
        W.boxp("stone", 6.5, y - 1.5, 8, y + 1.5, 0.0, 28.0)
        W.boxp("stone", 6.3, y - 1.8, 8, y + 1.8, 28.0, 0.6, solid=False)
    for y in [-58, -46, 8, 20, 32]:                    # tall lit windows
        W.box("glass", -51.95, y, 18.0, 0.06, 3.0, 17.0, solid=False)
        W.boxp("trim", -52.0, y - 1.7, -51.9, y - 1.5, 17.8, 17.4, solid=False)
        W.boxp("trim", -52.0, y + 1.5, -51.9, y + 1.7, 17.8, 17.4, solid=False)
    for y in [-56, -48, -4, 14, 28, 40]:
        W.box("glass", 7.95, y, 18.0, 0.06, 3.0, 17.0, solid=False)
    for (y, m) in [(-52, "banner_c"), (-10, "banner_p"), (14, "banner_c"), (28, "banner_p"), (46, "banner_c")]:
        banner(W, -51.97, y, 5.0, 11.0, 1, 0, m, w=3.6)
    for (y, m) in [(-58, "banner_p"), (-30, "banner_c"), (0, "banner_p"), (22, "banner_c"), (50, "banner_p")]:
        banner(W, 7.97, y, 5.0, 11.0, -1, 0, m, w=3.6)
    banner(W, -39.0, 57.95, 10.0, 26.0, 0, -1, "banner_p", w=0.1) if False else None
    for k, y in enumerate([-62, -46, -12, 12, 36, 52]):
        KP.brazier_big(W, -51.1, y, flame="flame" if k % 2 else "flame_m")
    for y in (-44, -14, 16, 46):
        KP.chandelier(W, -39.0, y, 31.0, r=3.4)
    for y in range(-58, 56, 12):                       # roof trusses
        W.boxp("wood", -52, y - 0.8, 8, y + 0.8, 36.5, 3.5, solid=False, tag="roof")
    # burgundy carpet (decals) and rune ring
    W.decal("carpet", -39.0, 13.0, 6.0, 84.0, z=0.012)
    W.decal("carpet", -47.0, -30.0, 16.0, 6.0, z=0.012)
    for x in (-42.15, -35.85):
        W.line("trim", x, -33, x, 55, 0.14, z=0.02)
    W.line("trim", -55, -33.15, -42, -33.15, 0.14, z=0.02)
    W.line("trim", -55, -26.85, -42, -26.85, 0.14, z=0.02)
    W.ring("rune", -39.0, 30.0, 8.0, 0.18, n=64, z=0.016)
    W.ring("rune", -39.0, 30.0, 7.5, 0.08, n=64, z=0.016)
    # plinth + the dragon statue (lies along the hall, nose to the entrance)
    W.boxp("stone", -21, -6, 6, 40, 0.0, 1.6)
    W.boxp("trim", -21.2, -6.2, 6.2, 40.2, 1.5, 0.12, solid=False)
    W.boxp("trim", -21.02, -6.02, 6.02, 40.02, 0.3, 0.12, solid=False)   # flush: the lidar sees this band
    W.boxp("stone", -20.0, -5.0, 5.0, 39.0, 1.6, 0.05, solid=False)
    place_statue(W, -7.5, 17.0, 1.65, -math.pi / 2, 12.0, 7.0, 12.0)
    for (x, y) in [(-19.0, -4.8), (4.0, -4.8), (-19.0, 38.8), (4.0, 38.8)]:
        KP.brazier_big(W, x, y, h=1.9, flame="flame_m")


class _Proxy:
    def __init__(self, W, cx, cy, z0, yaw, sx, sy, sz):
        self.W, self.cx, self.cy, self.z0, self.yaw, self.s = W, cx, cy, z0, yaw, (sx, sy, sz)
        self.c, self.sn = math.cos(yaw), math.sin(yaw)

    def T(self, p):
        x, y, z = p[0] * self.s[0], p[1] * self.s[1], p[2] * self.s[2]
        return (self.cx + x * self.c - y * self.sn, self.cy + x * self.sn + y * self.c, self.z0 + z)

    def add_mesh(self, mat, verts, faces, smooth=True, uv=None):
        self.W.add_mesh(mat, [self.T(v) for v in verts], faces, smooth)

    def ellipsoid(self, mat, c, radii, seg=12, rings=8, yaw=0.0, pitch=0.0, tile=None):
        r = radii[0] * self.s[0]
        self.W.ellipsoid(mat, self.T(c), (r, r, r), seg=seg, rings=rings)


def place_statue(W, cx, cy, z0, yaw, sx, sy, sz):
    dragon_statue.build(_Proxy(W, cx, cy, z0, yaw, sx, sy, sz), 0.0, 0.0, 0.0, yaw=0.0)


def hoard(W):
    for (x, y, r, h) in [(-14, 70, 5.0, 5.5), (-3, 80, 4.2, 4.5), (-20, 82, 3.0, 3.2), (2, 66, 2.6, 2.6), (-8, 62, 1.8, 1.6),
                         (-26, 72, 1.4, 1.4), (4, 76, 1.8, 1.8), (-12, 84, 1.5, 1.4)]:
        KP.gold_pile(W, x, y, r, h)
    for (x, y, yaw) in [(-22, 64, 0.4), (-6, 72, -0.7), (-17, 77, 1.6), (5, 71, 0.2), (-1, 62, 0.0), (-24, 80, 0.9), (-9, 67, 2.0), (3, 84, -0.3)]:
        KP.chest(W, x, y, yaw)
    for (x, y, r, h) in [(-24.5, 64, 1.3, 14), (-24.5, 76, 1.3, 16), (-24.5, 85, 1.0, 12), (6.5, 62, 1.4, 12), (6.5, 78, 1.8, 18), (-51.6, 62, 1.3, 15), (-51.6, 78, 1.3, 14),
                         (-9, 86.5, 1.8, 16)]:
        KP.stalagmite(W, x, y, r, h)
    for (x, y) in [(-51.5, 84), (-51.5, 70), (7, 84)]:
        KP.stalagmite(W, x, y, 0.9, 7)
    for k in range(70):                                 # scattered coins as decals
        x, y = rng.uniform(-26, 7), rng.uniform(60, 85)
        W.box("gold", x, y, 0.004, rng.uniform(0.2, 0.5), rng.uniform(0.2, 0.5), 0.02, yaw=rng.uniform(0, 3), solid=False)
    for (x, y) in [(-51.0, 66), (-51.0, 80), (7.4, 66), (7.4, 82)]:
        KP.brazier_big(W, x, y, h=1.8, flame="flame_m")
    for k in range(6):                                  # rib arches framing the lane in the cave mouth
        pass
    KP.rib_arch(W, -39.0, 62.0, 62.0 + 0.001, 0) if False else None


def obelisk(W, x, y, h=7.0):
    W.box("stone", x, y, 0.0, 1.2, 1.2, 1.0)
    W.frustum("stone", (x, y, 1.0), (x, y, h), 0.5, 0.1, seg=4)
    W.ellipsoid("rune", (x, y, h + 0.2), (0.16, 0.16, 0.26), seg=6, rings=4)


def landmarks(W):
    # hedge-planter flanks along the dragon lane (continuous, 3 m gaps): keeps lidar landmark density up inside the 22 m lane
    for (x0, x1) in [(-87.5, -77.0), (-74.0, -65.0), (-62.0, -57.5)]:
        KP.parterre(W, x0, -44.2, x1, -42.6)
    for (x0, x1) in [(-98.0, -90.0), (-87.0, -76.5), (-74.0, -65.0), (-62.0, -53.0)]:   # north lane flank, flush with the 20 m route (3 m gaps)
        KP.parterre(W, x0, -20.0, x1, -18.4)
    for x in (-84, -78, -72, -66, -60):
        obelisk(W, x, -48.0)
    for (y0, y1) in [(-63.0, -52.0), (-49.0, -42.6)]:
        KP.parterre(W, -88.8, y0, -87.2, y1)
    for (y0, y1) in [(-63.0, -52.0), (-49.0, -38.0), (-35.0, -24.0), (-21.0, -17.0)]:
        KP.parterre(W, -111.4, y0, -110.0, y1)     # west lane flank flush with the 20 m route
    for (x, y) in [(-47, -54), (-31, -54), (-39, -62), (-47, -62), (-31, -62)]:
        obelisk(W, x, y, 9.0)
    for (x, y) in [(-15.0, -52.0), (-5.0, -52.0), (-15.0, -40.0), (-5.0, -40.0), (-15.0, -28.0), (-5.0, -28.0), (-15.0, -16.0), (-5.0, -16.0)]:
        KP.knight_statue(W, x, y, math.pi / 2, 1.2)
    for (x0, x1) in [(-49.5, -42.0), (-36.0, -29.0)]:      # south hall flank (flush with the dragon lane edge)
        KP.parterre(W, x0, -41.6, x1, -40.0)

    # lidar landmarks in the margins of the 22 m lanes (dragon needs 20 m: posts sit in the outer 1 m)
    for x in range(-86, -57, 5):
        KP.lamp_post(W, x, -40.6)
    for x in (-92.0, -81.5, -69.5, -57.5):
        KP.lamp_post(W, x, -17.2)
    for y in (-44.0, -50.0):
        KP.lamp_post(W, -112.2, y)
    for y in (-18.5, -9.5, -1.0):
        KP.knight_statue(W, -50.9, y, 0.0, 1.0)
    for (x0, x1) in [(-56.5, -49.7)]:                         # south flank of the hall entrance
        KP.parterre(W, x0, -41.6, x1, -40.0)
    KP.parterre(W, -89.0, -41.6, -87.5, -40.0)               # inner corner of the lane junction
    for (x0, x1) in [(-70.0, -57.8)]:
        KP.parterre(W, x0, -41.9, x1, -40.3)

def courtyard(W):
    inL = lambda x0, y0, x1, y1: ((x0 < -89 + 2 and x1 > -111 - 2 and y1 > -64 and y0 < -19 + 2) or (x1 > -111 - 2 and x0 < -56 and y0 < -19 + 2 and y1 > -41 - 2 and y1 < 30 and x0 > -111 - 3 and y0 < -19 + 2 and y1 > -41 - 2 and False))
    # west garden: N-S beds
    for x in (-140.0, -130.0, -120.0):
        for y0 in (-60.0, -42.0, -24.0, -6.0, 12.0 - 6.0):
            if y0 + 12 > 17:
                continue
            KP.parterre(W, x - 1.2, y0, x + 1.2, y0 + 12)
    # middle gardens east of the lane, north of the dragon lane
    for (x0, y0) in [(-108, -16.7), (-91, -16.7), (-74, -16.7), (-108, 12.8), (-74, 12.8), (-108, -1.4), (-70, -1.4)]:
        if x0 > -75 and y0 < -10:
            continue
        KP.parterre(W, x0, y0, x0 + 12, y0 + 2.4)
    KP.fountain(W, -83.0, -1.0)
    for (x, y) in [(-102, 6), (-66, 7), (-102, -9), (-63, -12), (-142, 18)]:
        KP.tree(W, x, y)
    for (x, y, yaw) in [(-115.0, -58.0, 0), (-115.0, -46.0, 0), (-115.0, -34.0, 0), (-115.0, -22.0, 0), (-85.0, -58.0, math.pi), (-85.0, -46.0, math.pi),
                        (-70.0, -17.0, math.pi / 2), (-60.0, -17.0, math.pi / 2)]:
        KP.knight_statue(W, x, y, yaw, 1.0)
    for y in (-60, -44, -28, -12):
        KP.lamp_post(W, -112.6, y)
    for y in (-60, -44):
        KP.lamp_post(W, -87.4, y)
    # training yard (south east): hay, dummies, wagon, well, benches
    for (x, y) in [(-84, -60), (-82.9, -60), (-81.8, -60)]:
        W.box("hay", x, y, 0.0, 1.0, 0.8, 0.62)
    for (x, y) in [(-70, -62), (-70, -58), (-64, -60), (-76, -52), (-62, -50)]:
        W.cyl("wood", x, y, 0.0, 0.2, 2.0, seg=8)
        W.box("wood", x, y, 1.5, 1.3, 0.12, 0.12, solid=False)
        W.ellipsoid("hay", (x, y, 2.2), (0.2, 0.2, 0.22), seg=8, rings=5)
        W.box("leather", x, y, 0.9, 0.5, 0.5, 0.7, solid=False)
    for (x, y) in [(-78, -44), (-64, -44)]:
        W.box("wood", x, y, 0.0, 0.2, 0.2, 2.0)
        W.cyl("hay", x + 0.2, y, 1.1, 0.7, 0.1, seg=16, solid=False)
        W.cyl("banner_c", x + 0.2, y, 1.12, 0.45, 0.1, seg=16, solid=False)
    W.col_box(-60.0, -56.0, 0.0, 2.8, 1.8, 1.0)
    W.box("wood", -60.0, -56.0, 0.55, 2.7, 1.5, 0.45, solid=False)
    W.box("wood", -60.0, -56.0, 0.0, 2.4, 1.2, 0.55, solid=False)
    for sx_ in (-0.9, 0.9):
        for sy_ in (-0.9, 0.9):
            KP.wheel(W, (-60.0 + sx_, -56.0 + sy_, 0.55), 0.55, "y", "wood", "iron", thick=0.1)
    for (x, y) in [(-75.0, -62.0), (-79.0, -62.0)]:
        KP.barrel(W, x, y)
    KP.bench(W, -73.0, -44.0, 0.0)
    KP.bench(W, -68.0, -44.0, 0.0)
    # braziers along the walls and at the lane mouths
    for x in (-140, -128, -66, -58):
        KP.brazier_big(W, x, -62.0, flame="flame" if x % 2 else "flame_m")
    for y in (-52, -16, 12):
        KP.brazier_big(W, -144.5, y)
    for y in (-62, -50, -44):
        KP.brazier_big(W, -57.5, y, flame="flame_m")
    for y in (-16, 8, 18):
        KP.brazier_big(W, -57.5, y)
    for x in (-130, -108, -90, -70):
        KP.brazier_big(W, x, 19.0, flame="flame" if x % 4 else "flame_m")
    # solid pilasters on the courtyard faces
    for x in range(-142, -60, 12):
        if -113 < x < -87:
            continue
        W.boxp("wallg_out", x - 1.5, -64, x + 1.5, -62.5, 0.0, 26.0)
    for y in (-52, -22, 8):
        W.boxp("wallg_out", -146, y - 1.5, -144.5, y + 1.5, 0.0, 26.0)
    # decals: lane flagstones + gold inlay along the dragon lane
    for x in (-111.0, -89.0):
        W.line("trim", x, -64, x, -19, 0.2, z=0.012)
    W.line("trim", -89, -41, -56, -41, 0.2, z=0.012)
    W.line("trim", -89, -19, -56, -19, 0.2, z=0.012)
    W.ring("rune", -100.0, -50.0, 5.0, 0.2, n=48, z=0.014)
    W.decal("carpet", -100.0, -62.0, 5.0, 4.0, z=0.012)


def gatehouse(W):
    for (x, y) in [(-123, -86), (-123, -75), (-80, -86), (-80, -75)]:
        KP.long_table(W, x, y, 3.0, yaw=0.0)
    for (x, y, yaw) in [(-125.5, -80.8, 0.0), (-117.0, -84.0, 0.0), (-83.0, -80.8, 0.0), (-75.0, -75.0, 0.0)]:
        KP.rack(W, x, y, yaw, l=2.0)
    for (x, y) in [(-125.5, -87), (-76.0, -87.5), (-117.0, -72)]:
        KP.barrel(W, x, y)
    for (x, y) in [(-126.0, -73.0), (-74.0, -73.5)]:
        KP.crate(W, x, y, 0.8)
    for (x, y) in [(-100.0, -95.0)]:
        pass
    banner(W, -120, -91.0, 8.0, 20.0, 0, -1, "banner_c", w=6.0)
    banner(W, -80, -91.0, 8.0, 20.0, 0, -1, "banner_p", w=6.0)
    banner(W, -100, -91.0, 28.0, 7.0, 0, -1, "banner_c", w=14.0)
    for x in (-111.1, -88.9):
        W.box("trim", x, -90.0, 0.0, 0.2, 2.2, 26.0, solid=False)
    for (x, y) in [(-112.0, -100.0), (-88.0, -100.0)]:
        KP.knight_statue(W, x, y, 0.0)


def ward(W):
    # armoury A0 x[-146,-122] y[24,46]
    for k in range(4):
        KP.rack(W, -144.2, 28 + k * 4.5, math.pi / 2, l=2.4)
    for k in range(3):
        KP.rack(W, -133 + 0, 27 + k * 6.5, 0.0, l=2.4)
    KP.long_table(W, -128.5, 38, 3.4, yaw=math.pi / 2)
    for (x, y) in [(-140.0, 43.5), (-137.5, 43.5)]:
        KP.barrel(W, x, y)
    # forge A1 x[-122,-98]
    KP.furnace(W, -110.0, 44.4, math.pi)
    KP.anvil(W, -110.0, 41.0)
    KP.anvil(W, -104.0, 38.0, 0.5)
    for (x, y) in [(-119.0, 27.0), (-118.0, 28.2), (-100.0, 44.0), (-100.0, 42.8)]:
        KP.barrel(W, x, y)
    KP.long_table(W, -118.0, 36.0, 3.0, yaw=math.pi / 2)
    # kitchen A2 x[-98,-76]
    KP.long_table(W, -87.0, 35.0, 5.0)
    KP.long_table(W, -87.0, 30.0, 5.0)
    KP.furnace(W, -77.8, 44.6, math.pi / 2 + 1.0)
    for (x, y) in [(-96.5, 44.0), (-95.2, 44.0), (-96.0, 26.0), (-94.9, 27.0)]:
        KP.barrel(W, x, y)
    for (x, y) in [(-82.0, 26.0), (-80.5, 26.0)]:
        KP.crate(W, x, y, 0.8)
    # chapel A3 x[-76,-56]
    for k in range(4):
        KP.pew(W, -70.2, 29 + k * 3.4, 3.4, math.pi / 2)
        KP.pew(W, -61.8, 29 + k * 3.4, 3.4, math.pi / 2)
    W.box("stone", -66.0, 44.2, 0.0, 3.4, 1.4, 0.9)
    W.box("trim", -66.0, 44.2, 0.9, 3.6, 1.6, 0.06, solid=False)
    W.box("rune", -66.0, 45.3, 1.8, 2.2, 0.05, 1.6, solid=False)
    for x in (-72.0, -60.0):
        KP.brazier_big(W, x, 44.8, h=1.2, flame="flame_m")
    # B0 barracks x[-146,-122] y[50,66]
    for k in range(4):
        KP.bunk(W, -144.0, 52.4 + k * 3.2, math.pi / 2)
        KP.bunk(W, -125.0, 52.4 + k * 3.2, math.pi / 2)
    KP.long_table(W, -134.0, 60.0, 3.0, yaw=math.pi / 2)
    # tower C0 x[-146,-122] y[66,86]: newel and stair
    W.cyl("stone", -134.0, 76.0, 0.0, 1.8, 5.9, seg=16)
    for k in range(10):
        a = k * 0.6
        W.box("stone", -134.0 + 1.5 * math.cos(a), 76.0 + 1.5 * math.sin(a), 0.8 + k * 0.5, 1.6, 0.9, 0.2, yaw=a, solid=False)
    KP.brazier_big(W, -144.0, 68.0, h=1.2)
    # crypt ring corridor around the ossuary block, sarcophagi
    W.boxp("stone", -118.8, 53.2, -101.2, 82.8, 0.0, 2.8)
    W.boxp("trim", -119.0, 53.0, -101.0, 83.0, 2.8, 0.15, solid=False)
    for k in range(6):
        W.box("rune", -110.0 + (k - 2.5) * 2.4, 53.14, 1.1, 0.9, 0.04, 1.2, solid=False)
    for (x, y, yaw) in [(-120.4, 70.0, math.pi / 2), (-99.6, 62.0, math.pi / 2), (-99.6, 76.0, math.pi / 2)]:
        pass
    for (x, y) in [(-120.5, 58), (-120.5, 77), (-99.5, 58), (-99.5, 77)]:
        KP.brazier_big(W, x, y, h=1.0, flame="flame_m")
    for k in range(5):
        KP.sarcophagus(W, -110.0 + (k - 2) * 3.4, 84.9 - 0.5, 0.0) if False else None
    # B2 servants x[-98,-76] y[50,66], C2 larder, B3 store, C3 tower
    for k in range(3):
        KP.bunk(W, -96.6, 52.4 + k * 3.8, math.pi / 2)
        KP.bunk(W, -77.4, 52.4 + k * 3.8, math.pi / 2)
    KP.long_table(W, -87.0, 61.0, 3.0, yaw=0.0)
    for (x, y) in [(-96.8, 84.0), (-95.5, 84.0), (-94.2, 84.0), (-78.0, 84.0), (-79.3, 84.0)]:
        KP.barrel(W, x, y)
    for (x, y) in [(-96.8, 70.0), (-96.8, 71.2), (-78.0, 70.0)]:
        KP.crate(W, x, y, 0.8)
    for (x, y) in [(-74.0, 52.5), (-74.0, 53.7), (-73.0, 62.0), (-58.0, 52.5), (-58.0, 53.7), (-58.0, 64.0)]:
        KP.crate(W, x, y, 0.9)
    for k in range(3):
        KP.long_table(W, -66.0, 55.0 + k * 3.5, 4.0)
    W.cyl("stone", -66.0, 76.0, 0.0, 1.8, 5.9, seg=16)
    for k in range(10):
        a = k * 0.6 + 1.0
        W.box("stone", -66.0 + 1.5 * math.cos(a), 76.0 + 1.5 * math.sin(a), 0.8 + k * 0.5, 1.6, 0.9, 0.2, yaw=a, solid=False)
    KP.brazier_big(W, -74.0, 84.0, h=1.2)
    # corridor
    for x in (-141.0, -100.0, -80.0):
        KP.brazier_big(W, x, 49.0, h=1.0, flame="flame_m")
    for x in (-138.0, -118.0, -96.0, -70.0):
        banner(W, x, 46.3, 1.8, 3.2, 0, 1, "banner_c", w=1.2)
    # ward torches on the walls
    for (x, y, nx, ny) in [(-145.8, 30, 1, 0), (-145.8, 40, 1, 0), (-145.8, 55, 1, 0), (-145.8, 80, 1, 0), (-56.2, 60, -1, 0), (-56.2, 70, -1, 0)]:
        torch(W, x, y, 3.0, nx, ny)


def grounds(W):
    # forecourt and road: lamp posts, trees in planters, benches, obelisks, knights
    for x in range(-136, 16, 16):
        KP.lamp_post(W, x, -106.5)
        KP.lamp_post(W, x, -93.5 if x > -72 or x < -130 else -93.5)
    for x in range(-128, 12, 24):
        KP.tree(W, x, -102.0, h=6.0)
        KP.bench(W, x + 8, -106.6, 0.0)
        KP.bench(W, x + 8, -92.6, 0.0)
        KP.parterre(W, x + 4, -100.6, x + 14, -98.2, n=5)
    for (x, y) in [(-100.0, -105.0), (-120.0, -97.0), (-80.0, -97.0), (-60.0, -105.0)]:
        W.box("stone", x, y, 0.0, 1.4, 1.4, 1.0)
        W.frustum("stone", (x, y, 1.0), (x, y, 7.0), 0.5, 0.12, seg=4)
        W.ellipsoid("rune", (x, y, 7.2), (0.2, 0.2, 0.3), seg=6, rings=4)
    # promenade
    for y in range(-84, 68, 16):
        KP.lamp_post(W, 13.8, y)
        KP.lamp_post(W, 18.2, y + 8) if False else None
    for y in range(-76, 64, 24):
        KP.bench(W, 18.0, y, math.pi / 2, l=2.2)
        KP.knight_statue(W, 14.2, y + 10, 0.0)
    for y in range(-68, 60, 32):
        KP.brazier_big(W, 17.9, y, flame="flame_m")
    # banks: lamp posts, benches, trees, planters on the south / north / east strips
    for x in range(34, 182, 16):
        for yy in (-66.5, 66.5):
            KP.lamp_post(W, x, yy)
        KP.bench(W, x + 8, -65.8, 0.0)
        KP.bench(W, x + 8, 65.8, 0.0)
        if x % 32 == 2:
            KP.tree(W, x + 8, -60.0, h=6.5)
            KP.tree(W, x + 8, 60.0, h=6.5)
        KP.parterre(W, x + 2, -63.0, x + 12, -61.0, n=5) if x % 32 == 2 else None
        KP.parterre(W, x + 2, 61.0, x + 11, 63.0, n=5) if x % 32 == 2 else None
    for y in range(-44, 50, 16):
        KP.lamp_post(W, 180.0, y)
        KP.bench(W, 179.0, y + 8, math.pi / 2)
        KP.tree(W, 172.0, y + 4, h=6.0) if y % 32 == 4 else None
    for (x, y) in [(60.0, -60.0), (110.0, 60.0), (150.0, -60.0), (173.0, 28.0), (173.0, -28.0), (60.0, 60.0)]:
        KP.fountain(W, x, y, r=3.2)
    for (x, y) in [(40.0, -60.0), (80.0, 60.0), (130.0, -60.0), (173.0, 12.0), (173.0, -12.0), (100.0, 60.0)]:
        KP.knight_statue(W, x, y, 0.0, 1.0)


def lights(W):
    L = W.light
    big = (0.5, 0.02, 0.002)
    for i, (x, y) in enumerate([(-130, -50), (-130, -10), (-130, 10), (-70, -50), (-70, 0), (-100, 10), (-100, -30), (-60, -30)]):
        L("court_%d" % i, x, y, 5.0, (1.0, 0.55, 0.25) if i % 2 else (0.7, 0.4, 1.0), 1.2, 38, 3000, atten=big)
    for i, y in enumerate([-50, -25, 0, 25, 50]):
        L("hall_w%d" % i, -46, y, 12.0, (1.0, 0.55, 0.25), 2.0, 55, 8000, atten=big)
        L("hall_e%d" % i, 2, y, 12.0, (0.8, 0.4, 1.0), 2.0, 55, 8000, atten=big)
    for i, y in enumerate([-35, 5, 40]):
        L("hall_c%d" % i, -39, y, 26.0, (1.0, 0.6, 0.35), 3.0, 70, 12000, atten=big)
    L("dragon_glow", -7.5, 17, 12.0, (0.7, 0.2, 1.0), 3.0, 60, 12000, atten=big)
    L("hoard_a", -14, 72, 8.0, (1.0, 0.75, 0.3), 3.5, 45, 9000, atten=big)
    L("hoard_b", -40, 72, 12.0, (1.0, 0.6, 0.3), 2.0, 45, 6000, atten=big)
    L("hoard_c", 0, 80, 6.0, (1.0, 0.7, 0.3), 2.0, 30, 5000, atten=big)
    for i, (x, y) in enumerate([(-100, -105), (-60, -100), (-20, -100), (-120, -80), (-80, -80), (-100, -75)]):
        L("gate_%d" % i, x, y, 6.0 if y < -90 else 4.0, (1.0, 0.6, 0.3), 1.4, 30, 3000, atten=big)
    for i, (x, y) in enumerate([(-134, 35), (-110, 35), (-87, 35), (-66, 35), (-134, 58), (-120, 68), (-100, 68), (-87, 58), (-66, 58), (-134, 76), (-66, 76), (-87, 76), (-100, 48), (-134, 48), (-70, 48)]):
        L("ward_%d" % i, x, y, 3.8, (1.0, 0.55, 0.25) if i % 3 else (0.6, 0.3, 1.0), 1.4, 16, 800, atten=(0.4, 0.05, 0.01))
    for i, (x, y) in enumerate([(16, -80), (16, -40), (16, 0), (16, 40), (16, 62), (50, -60), (100, -60), (150, -60), (173, -20), (173, 30), (150, 60), (100, 60), (50, 60)]):
        L("walk_%d" % i, x, y, 6.0, (1.0, 0.6, 0.35) if i % 2 else (0.7, 0.45, 1.0), 1.6, 36, 2500, atten=big)
    for i, (x, y, z) in enumerate([(70, 0, -9), (95, -14, -12), (105, 6, -10), (118, 0, -12), (50, 18, -6)]):
        L("deep_%d" % i, x, y, z, (0.2, 0.55, 0.8), 1.8, 40, 3000, atten=big)


def route_check(W):
    """Every collision primitive must stay clear of the dragon route (>= ROUTE_HALF either side, z < 24)."""
    bad = []
    for p in W.prims:
        px, py, pz = p["pose"][0], p["pose"][1], p["pose"][2]
        if p["type"] == "box":
            hx, hy, h = p["size"][0] / 2, p["size"][1] / 2, p["size"][2]
            yw = p["pose"][3]
            r = abs(hx * math.cos(yw)) + abs(hy * math.sin(yw)), abs(hx * math.sin(yw)) + abs(hy * math.cos(yw))
            x0, x1, y0, y1 = px - r[0], px + r[0], py - r[1], py + r[1]
        else:
            rr = p["radius"]
            h = p["length"]
            x0, x1, y0, y1 = px - rr, px + rr, py - rr, py + rr
        zlo, zhi = pz - h / 2 if p["type"] == "cylinder" else pz - p["size"][2] / 2, pz + h / 2 if p["type"] == "cylinder" else pz + p["size"][2] / 2
        if zhi < 0.0 or zlo > 24.0:
            continue
        for (ax, ay), (bx, by) in zip(ROUTE[:-1], ROUTE[1:]):
            qx0, qx1 = min(ax, bx) - ROUTE_HALF, max(ax, bx) + ROUTE_HALF
            qy0, qy1 = min(ay, by) - ROUTE_HALF, max(ay, by) + ROUTE_HALF
            if x1 > qx0 + 1e-6 and x0 < qx1 - 1e-6 and y1 > qy0 + 1e-6 and y0 < qy1 - 1e-6 and zhi > 0.3:
                bad.append((round(px, 1), round(py, 1), p["type"]))
                break
    return bad


def build(W):
    floors(W)
    tall_walls(W)
    gatehouse(W)
    hall(W)
    hoard(W)
    courtyard(W)
    landmarks(W)
    ward(W)
    grounds(W)
    KL.quay(W)
    KL.bed_mesh(W)
    KL.bed_collision(W)
    KL.water_surface(W)
    KL.temple_ruins(W)
    KL.sky(W)
    lights(W)
    bad = route_check(W)
    print("DRAGON_ROUTE_CHECK intrusions:", len(bad), bad)
