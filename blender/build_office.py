"""Generate a realistic open-plan office scene for semantic robot navigation.

Run:  blender -b --python build_office.py
Writes office.blend + scene_objects.json in the project directory.
"""

import bpy, bmesh, os, math, json, random
from mathutils import Vector

BASE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(BASE, "assets")
random.seed(7)

R = 10.0           # room half-size (inner faces at +/- R)
WALL_T = 0.2       # wall thickness
CEIL_H = 3.2       # ceiling height
SILL, HEAD = 0.9, 2.4   # window sill / head height

LANDMARKS = []     # semantic manifest entries

# Rooms are listed most-specific first so zone_at() resolves overlaps sensibly.
ROOMS = [
    {"name": "conference_room", "label": "conference room",
     "bounds": [[3.0, 3.0], [10.0, 10.0]], "centre": [6.5, 6.4], "door": [3.7, 3.0]},
    {"name": "private_office", "label": "private office",
     "bounds": [[-10.0, 4.6], [-4.4, 10.0]], "centre": [-7.2, 7.3], "door": [-5.4, 4.6]},
    {"name": "break_area", "label": "break room / kitchenette",
     "bounds": [[-4.3, 4.7], [2.9, 10.0]], "centre": [-0.7, 7.4]},
    {"name": "lounge", "label": "lounge seating area",
     "bounds": [[4.6, -4.2], [9.8, 0.9]], "centre": [6.8, -1.6]},
    {"name": "reception", "label": "reception / entrance",
     "bounds": [[-4.2, -10.0], [3.4, -6.4]], "centre": [0.0, -8.6]},
    {"name": "open_plan", "label": "open-plan work area",
     "bounds": [[-10.0, -10.0], [10.0, 4.6]], "centre": [-4.0, -2.0]},
]


def zone_at(x, y):
    for r in ROOMS:
        (x0, y0), (x1, y1) = r["bounds"]
        if x0 <= x <= x1 and y0 <= y <= y1:
            return r["name"]
    return ""



# ---------------------------------------------------------------------------
# Scene / collection setup
# ---------------------------------------------------------------------------

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

COLLS = {}
for cname in ("Shell", "Lighting", "Furniture", "Props", "NavMarkers", "Exterior", "Cameras"):
    c = bpy.data.collections.new(cname)
    scene.collection.children.link(c)
    COLLS[cname] = c


def link(ob, coll="Furniture"):
    COLLS[coll].objects.link(ob)
    return ob


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def add_box(name, size, loc, rot=(0, 0, 0), mat=None, coll="Furniture"):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector(size), verts=bm.verts)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = rot
    if mat:
        me.materials.append(mat)
    return link(ob, coll)


def add_cyl(name, radius, depth, loc, rot=(0, 0, 0), verts=32, mat=None,
            coll="Furniture", smooth=False):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=verts,
                          radius1=radius, radius2=radius, depth=depth)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = rot
    if mat:
        me.materials.append(mat)
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
    return link(ob, coll)


def add_cone(name, radius, depth, loc, rot=(0, 0, 0), verts=32, mat=None, coll="Furniture"):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=verts,
                          radius1=radius, radius2=0.0, depth=depth)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = rot
    if mat:
        me.materials.append(mat)
    return link(ob, coll)


def add_sphere(name, radius, loc, scale=(1, 1, 1), rot=(0, 0, 0), mat=None, coll="Furniture"):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=2, radius=radius)
    bmesh.ops.scale(bm, vec=Vector(scale), verts=bm.verts)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = rot
    if mat:
        me.materials.append(mat)
    for p in me.polygons:
        p.use_smooth = True
    return link(ob, coll)


def add_plane(name, size, loc, rot=(0, 0, 0), mat=None, coll="Shell"):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=0.5)
    bmesh.ops.scale(bm, vec=Vector((size[0], size[1], 1.0)), verts=bm.verts)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = rot
    if mat:
        me.materials.append(mat)
    return link(ob, coll)


def bevel(ob, width=0.006, segments=2, angle=55.0):
    m = ob.modifiers.new("Bevel", 'BEVEL')
    m.width = width
    m.segments = segments
    m.limit_method = 'ANGLE'
    m.angle_limit = math.radians(angle)
    return ob


def L(cx, cy, rot, lx, ly, lz):
    """Local (lx,ly,lz) around anchor (cx,cy) yawed by rot -> world location."""
    c, s = math.cos(rot), math.sin(rot)
    return (cx + lx * c - ly * s, cy + lx * s + ly * c, lz)


def group(name, parts, anchor):
    """Parent parts to an empty so each furniture item has one clean handle.

    matrix_world on a freshly created empty is still identity until the depsgraph
    updates, so the parent inverse is built explicitly from the anchor instead.
    """
    from mathutils import Matrix
    loc = Vector((anchor[0], anchor[1], anchor[2] if len(anchor) > 2 else 0.0))
    emp = bpy.data.objects.new(name, None)
    emp.empty_display_size = 0.25
    emp.location = loc
    link(emp, "Furniture")
    inv = Matrix.Translation(-loc)
    for p in parts:
        p.parent = emp
        p.matrix_parent_inverse = inv
    return emp


def landmark(name, label, category, loc, size=(0.5, 0.5, 0.5), zone="", target=False):
    LANDMARKS.append({
        "name": name, "semantic_label": label, "category": category, "zone": zone,
        "location": [round(float(v), 3) for v in loc],
        "size": [round(float(v), 3) for v in size],
        "is_target": target,
    })


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

def set_in(node, key, value):
    if key in node.inputs:
        node.inputs[key].default_value = value


def new_mat(name, base=(0.8, 0.8, 0.8), rough=0.5, metal=0.0, emission=None,
            emit_strength=0.0, transmission=0.0, ior=1.45):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    b = mat.node_tree.nodes["Principled BSDF"]
    set_in(b, "Base Color", (*base, 1.0))
    set_in(b, "Roughness", rough)
    set_in(b, "Metallic", metal)
    set_in(b, "IOR", ior)
    if transmission:
        set_in(b, "Transmission Weight", transmission)
        for attr in ("use_raytrace_refraction", "use_screen_refraction"):
            if hasattr(mat, attr):
                setattr(mat, attr, True)
    if emission:
        set_in(b, "Emission Color", (*emission, 1.0))
        set_in(b, "Emission Strength", emit_strength)
    return mat


def _coord(nt):
    return nt.nodes.new("ShaderNodeTexCoord")


def noise_bump_mat(name, base, rough, bump_scale, bump_strength, bump_dist,
                   var_scale=0.0, var_contrast=0.12, metal=0.0):
    """Solid colour with fine procedural surface detail (carpet, paint, fabric)."""
    mat = new_mat(name, base, rough=rough, metal=metal)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)

    n = nt.nodes.new("ShaderNodeTexNoise")
    n.inputs["Scale"].default_value = bump_scale
    n.inputs["Detail"].default_value = 6.0
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])

    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = bump_strength
    bump.inputs["Distance"].default_value = bump_dist
    nt.links.new(n.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])

    if var_scale:
        n2 = nt.nodes.new("ShaderNodeTexNoise")
        n2.inputs["Scale"].default_value = var_scale
        nt.links.new(tc.outputs["Object"], n2.inputs["Vector"])
        ramp = nt.nodes.new("ShaderNodeValToRGB")
        lo = tuple(max(0.0, c * (1 - var_contrast)) for c in base)
        hi = tuple(min(1.0, c * (1 + var_contrast)) for c in base)
        ramp.color_ramp.elements[0].color = (*lo, 1.0)
        ramp.color_ramp.elements[1].color = (*hi, 1.0)
        nt.links.new(n2.outputs["Fac"], ramp.inputs["Fac"])
        nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    return mat


def wood_mat(name, c_dark, c_light, grain_scale=9.0, stretch=(1.0, 0.06, 1.0), rough=0.32):
    """Subtle laminate/veneer: fine bands across X stretched along the grain (Y)."""
    mat = new_mat(name, c_light, rough=rough)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)
    mapn = nt.nodes.new("ShaderNodeMapping")
    mapn.inputs["Scale"].default_value = stretch
    nt.links.new(tc.outputs["Object"], mapn.inputs["Vector"])

    # anisotropic noise, not wave bands: periodic bands read as corduroy on big flat tops
    grain = nt.nodes.new("ShaderNodeTexNoise")
    grain.inputs["Scale"].default_value = grain_scale
    grain.inputs["Detail"].default_value = 9.0
    grain.inputs["Roughness"].default_value = 0.62
    nt.links.new(mapn.outputs["Vector"], grain.inputs["Vector"])

    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.32
    ramp.color_ramp.elements[1].position = 0.68
    ramp.color_ramp.elements[0].color = (*c_dark, 1.0)
    ramp.color_ramp.elements[1].color = (*c_light, 1.0)
    nt.links.new(grain.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])

    return mat


def tile_mat(name, c_tile, c_seam, tile=0.6, seam=0.02, rough=0.6):
    """Ceiling / floor tiles: brick texture with square bricks = tile grid."""
    mat = new_mat(name, c_tile, rough=rough)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)
    brick = nt.nodes.new("ShaderNodeTexBrick")
    brick.offset = 0.0
    brick.inputs["Scale"].default_value = 1.0
    brick.inputs["Color1"].default_value = (*c_tile, 1.0)
    brick.inputs["Color2"].default_value = (*[min(1, c * 1.03) for c in c_tile], 1.0)
    brick.inputs["Mortar"].default_value = (*c_seam, 1.0)
    brick.inputs["Mortar Size"].default_value = seam
    brick.inputs["Brick Width"].default_value = tile
    brick.inputs["Row Height"].default_value = tile
    nt.links.new(tc.outputs["Object"], brick.inputs["Vector"])
    nt.links.new(brick.outputs["Color"], b.inputs["Base Color"])
    return mat


def screen_mat(name):
    mat = new_mat(name, (0.02, 0.02, 0.03), rough=0.25)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)
    grad = nt.nodes.new("ShaderNodeTexGradient")
    grad.gradient_type = 'LINEAR'
    mapn = nt.nodes.new("ShaderNodeMapping")
    mapn.inputs["Rotation"].default_value = (0, 0, math.radians(90))
    nt.links.new(tc.outputs["Object"], mapn.inputs["Vector"])
    nt.links.new(mapn.outputs["Vector"], grad.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.02, 0.08, 0.22, 1.0)
    ramp.color_ramp.elements[1].color = (0.15, 0.35, 0.62, 1.0)
    nt.links.new(grad.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], b.inputs["Emission Color"])
    set_in(b, "Emission Strength", 1.6)
    return mat


def carpet_mat(name, base, tile=0.5):
    """Carpet tiles: fibre bump + mottling + faint alternating tile tone."""
    mat = new_mat(name, base, rough=0.97)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)

    fibre = nt.nodes.new("ShaderNodeTexNoise")
    fibre.inputs["Scale"].default_value = 210.0
    fibre.inputs["Detail"].default_value = 5.0
    nt.links.new(tc.outputs["Object"], fibre.inputs["Vector"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.55
    bump.inputs["Distance"].default_value = 0.002
    nt.links.new(fibre.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])

    mottle = nt.nodes.new("ShaderNodeTexNoise")
    mottle.inputs["Scale"].default_value = 22.0
    mottle.inputs["Detail"].default_value = 8.0
    nt.links.new(tc.outputs["Object"], mottle.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (*[c * 0.62 for c in base], 1.0)
    ramp.color_ramp.elements[1].color = (*[min(1, c * 1.45) for c in base], 1.0)
    nt.links.new(mottle.outputs["Fac"], ramp.inputs["Fac"])

    chk = nt.nodes.new("ShaderNodeTexChecker")
    chk.inputs["Color1"].default_value = (1.0, 1.0, 1.0, 1.0)
    chk.inputs["Color2"].default_value = (0.93, 0.93, 0.93, 1.0)
    chk.inputs["Scale"].default_value = 1.0 / tile
    nt.links.new(tc.outputs["Object"], chk.inputs["Vector"])

    mix = nt.nodes.new("ShaderNodeMixRGB")
    mix.blend_type = 'MULTIPLY'
    mix.inputs["Fac"].default_value = 1.0
    nt.links.new(ramp.outputs["Color"], mix.inputs["Color1"])
    nt.links.new(chk.outputs["Color"], mix.inputs["Color2"])
    nt.links.new(mix.outputs["Color"], b.inputs["Base Color"])
    return mat


M = {}
M['carpet'] = carpet_mat("carpet_office", (0.093, 0.090, 0.086))
M['carpet_lounge'] = noise_bump_mat("carpet_lounge_rug", (0.125, 0.092, 0.068), 0.95, 120, 0.5, 0.004,
                                    var_scale=5.0, var_contrast=0.18)
M['vinyl'] = tile_mat("floor_vinyl_break", (0.335, 0.325, 0.300), (0.225, 0.220, 0.205), tile=0.45,
                      seam=0.008, rough=0.28)
M['wall'] = noise_bump_mat("wall_paint", (0.50, 0.485, 0.45), 0.88, 90, 0.12, 0.0012)
M['wall_accent'] = noise_bump_mat("wall_paint_accent", (0.048, 0.098, 0.122), 0.85, 90, 0.12, 0.0012)
M['ceiling'] = tile_mat("ceiling_tile", (0.70, 0.70, 0.69), (0.47, 0.47, 0.46), tile=0.6, seam=0.012, rough=0.9)
M['skirting'] = new_mat("skirting", (0.22, 0.22, 0.23), rough=0.4)
M['wood_desk'] = noise_bump_mat("laminate_desk_grey", (0.345, 0.338, 0.325), 0.36, 55, 0.12, 0.0009,
                                var_scale=7.0, var_contrast=0.06)
M['wood_dark'] = wood_mat("wood_walnut", (0.062, 0.038, 0.025), (0.105, 0.066, 0.043),
                          grain_scale=11.0, rough=0.30)
M['wood_light'] = wood_mat("wood_birch", (0.330, 0.255, 0.170), (0.415, 0.330, 0.225),
                           grain_scale=9.0, rough=0.38)
M['metal'] = new_mat("metal_brushed", (0.42, 0.43, 0.45), rough=0.33, metal=1.0)
M['metal_dark'] = new_mat("metal_dark", (0.09, 0.09, 0.10), rough=0.42, metal=0.85)
M['alu'] = new_mat("aluminium_frame", (0.62, 0.63, 0.65), rough=0.28, metal=1.0)
M['plastic_black'] = new_mat("plastic_black", (0.035, 0.035, 0.04), rough=0.45)
M['plastic_white'] = new_mat("plastic_white", (0.78, 0.78, 0.77), rough=0.35)
M['fabric_blue'] = noise_bump_mat("fabric_chair_charcoal", (0.026, 0.030, 0.040), 0.94, 330, 0.55, 0.0016)
M['fabric_grey'] = noise_bump_mat("fabric_sofa_grey", (0.105, 0.110, 0.120), 0.93, 260, 0.5, 0.0018)
M['fabric_teal'] = noise_bump_mat("fabric_accent_teal", (0.040, 0.072, 0.076), 0.92, 280, 0.5, 0.0016)
M['glass'] = new_mat("glass_clear", (0.92, 0.96, 0.96), rough=0.02, transmission=1.0, ior=1.45)
M['screen'] = screen_mat("screen_display")
M['whiteboard'] = new_mat("whiteboard", (0.93, 0.93, 0.92), rough=0.08)
M['paper'] = new_mat("paper", (0.88, 0.88, 0.86), rough=0.7)
M['leaf'] = noise_bump_mat("plant_leaf", (0.055, 0.19, 0.055), 0.6, 80, 0.4, 0.003, var_scale=9.0, var_contrast=0.4)
M['pot'] = new_mat("plant_pot", (0.32, 0.24, 0.19), rough=0.65)
M['soil'] = new_mat("soil", (0.07, 0.055, 0.045), rough=0.95)
M['panel_light'] = new_mat("light_panel_emissive", (1.0, 0.97, 0.92), rough=0.5,
                           emission=(1.0, 0.92, 0.82), emit_strength=2.0)
M['concrete'] = noise_bump_mat("exterior_ground", (0.24, 0.25, 0.23), 0.95, 20, 0.3, 0.02)
M['building'] = new_mat("exterior_building", (0.30, 0.31, 0.33), rough=0.45)

# nav marker materials (colours + textures)
M['nav_red'] = new_mat("nav_cube_red", (0.62, 0.045, 0.045), rough=0.42)
M['nav_blue'] = new_mat("nav_cube_blue_glossy", (0.03, 0.09, 0.55), rough=0.1, metal=0.5)
M['nav_orange'] = new_mat("nav_cone_orange", (0.85, 0.24, 0.02), rough=0.45)
M['nav_yellow'] = new_mat("nav_cone_yellow", (0.85, 0.70, 0.02), rough=0.45)
M['nav_white'] = new_mat("nav_cone_white", (0.85, 0.85, 0.85), rough=0.4)
M['nav_red_cone'] = new_mat("nav_cone_red", (0.70, 0.03, 0.03), rough=0.45)


def checker_mat(name, c1, c2, scale=9.0):
    mat = new_mat(name, c1, rough=0.4)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)
    chk = nt.nodes.new("ShaderNodeTexChecker")
    chk.inputs["Color1"].default_value = (*c1, 1.0)
    chk.inputs["Color2"].default_value = (*c2, 1.0)
    chk.inputs["Scale"].default_value = scale
    nt.links.new(tc.outputs["Object"], chk.inputs["Vector"])
    nt.links.new(chk.outputs["Color"], b.inputs["Base Color"])
    return mat


def marble_mat(name, c1, c2):
    mat = new_mat(name, c1, rough=0.22)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)
    n = nt.nodes.new("ShaderNodeTexNoise")
    n.inputs["Scale"].default_value = 9.0
    n.inputs["Detail"].default_value = 8.0
    nt.links.new(tc.outputs["Object"], n.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (*c1, 1.0)
    ramp.color_ramp.elements[1].color = (*c2, 1.0)
    nt.links.new(n.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    return mat


def stripe_mat(name, c1, c2):
    mat = new_mat(name, c1, rough=0.45)
    nt = mat.node_tree
    b = nt.nodes["Principled BSDF"]
    tc = _coord(nt)
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.wave_type = 'BANDS'
    wave.inputs["Scale"].default_value = 7.0
    nt.links.new(tc.outputs["Object"], wave.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.interpolation = 'CONSTANT'
    ramp.color_ramp.elements[0].color = (*c1, 1.0)
    ramp.color_ramp.elements[1].color = (*c2, 1.0)
    nt.links.new(wave.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    return mat


M['nav_checker'] = checker_mat("nav_cube_checker_green", (0.02, 0.02, 0.02), (0.06, 0.42, 0.09))
M['nav_marble'] = marble_mat("nav_cube_marble_orange", (0.75, 0.28, 0.03), (0.92, 0.90, 0.86))
M['nav_stripe'] = stripe_mat("nav_cube_stripe_purple", (0.28, 0.04, 0.38), (0.86, 0.82, 0.90))


# ---------------------------------------------------------------------------
# Room shell
# ---------------------------------------------------------------------------

add_plane("floor_carpet", (2 * R, 2 * R), (0, 0, 0), mat=M['carpet'])
add_plane("ceiling", (2 * R, 2 * R), (0, 0, CEIL_H), rot=(math.pi, 0, 0), mat=M['ceiling'])


def wall_with_openings(name, axis, pos, span, openings, mat=M['wall'], height=CEIL_H):
    """Build a wall along `axis` ('x' or 'y') at coordinate `pos`, spanning `span`,
    leaving rectangular openings [(centre, width, z0, z1), ...]."""
    parts = []
    cuts = sorted([(c - w / 2, c + w / 2, z0, z1) for c, w, z0, z1 in openings])
    # full-height solid segments between openings
    edges = [span[0]] + [v for cut in cuts for v in cut[:2]] + [span[1]]
    for i in range(0, len(edges) - 1, 2):
        a, b = edges[i], edges[i + 1]
        if b - a > 1e-4:
            mid, ln = (a + b) / 2, b - a
            size = (ln, WALL_T, height) if axis == 'x' else (WALL_T, ln, height)
            loc = (mid, pos, height / 2) if axis == 'x' else (pos, mid, height / 2)
            parts.append(add_box(f"{name}_seg{i}", size, loc, mat=mat, coll="Shell"))
    # strips above / below each opening
    for j, (a, b, z0, z1) in enumerate(cuts):
        mid, ln = (a + b) / 2, b - a
        for k, (zz0, zz1) in enumerate([(0.0, z0), (z1, height)]):
            if zz1 - zz0 > 1e-4:
                size = (ln, WALL_T, zz1 - zz0) if axis == 'x' else (WALL_T, ln, zz1 - zz0)
                loc = (mid, pos, (zz0 + zz1) / 2) if axis == 'x' else (pos, mid, (zz0 + zz1) / 2)
                parts.append(add_box(f"{name}_op{j}_{k}", size, loc, mat=mat, coll="Shell"))
    return parts


def window_glazing(name, axis, pos, centre, width, z0=SILL, z1=HEAD, mullions=2):
    """Frame + glass filling an opening."""
    h = z1 - z0
    fr = 0.06
    if axis == 'x':
        add_box(f"{name}_frame_b", (width, WALL_T, fr), (centre, pos, z0 + fr / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_frame_t", (width, WALL_T, fr), (centre, pos, z1 - fr / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_frame_l", (fr, WALL_T, h), (centre - width / 2 + fr / 2, pos, (z0 + z1) / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_frame_r", (fr, WALL_T, h), (centre + width / 2 - fr / 2, pos, (z0 + z1) / 2), mat=M['alu'], coll="Shell")
        for i in range(1, mullions):
            x = centre - width / 2 + i * width / mullions
            add_box(f"{name}_mul{i}", (0.04, WALL_T * 0.9, h), (x, pos, (z0 + z1) / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_glass", (width - 2 * fr, 0.012, h - 2 * fr), (centre, pos, (z0 + z1) / 2), mat=M['glass'], coll="Shell")
    else:
        add_box(f"{name}_frame_b", (WALL_T, width, fr), (pos, centre, z0 + fr / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_frame_t", (WALL_T, width, fr), (pos, centre, z1 - fr / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_frame_l", (WALL_T, fr, h), (pos, centre - width / 2 + fr / 2, (z0 + z1) / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_frame_r", (WALL_T, fr, h), (pos, centre + width / 2 - fr / 2, (z0 + z1) / 2), mat=M['alu'], coll="Shell")
        for i in range(1, mullions):
            y = centre - width / 2 + i * width / mullions
            add_box(f"{name}_mul{i}", (WALL_T * 0.9, 0.04, h), (pos, y, (z0 + z1) / 2), mat=M['alu'], coll="Shell")
        add_box(f"{name}_glass", (0.012, width - 2 * fr, h - 2 * fr), (pos, centre, (z0 + z1) / 2), mat=M['glass'], coll="Shell")


WALL_OUT = R + WALL_T / 2   # wall centre-line so inner face sits exactly at +/-R

# East wall: 4 windows
east_win = [-6.0, -2.0, 2.0, 6.0]
wall_with_openings("wall_east", 'y', WALL_OUT, (-R - WALL_T, R + WALL_T),
                   [(c, 2.4, SILL, HEAD) for c in east_win])
for i, c in enumerate(east_win):
    window_glazing(f"win_e{i}", 'y', WALL_OUT, c, 2.4)

# North wall: 3 windows
north_win = [-7.0, 0.0, 7.0]
wall_with_openings("wall_north", 'x', WALL_OUT, (-R - WALL_T, R + WALL_T),
                   [(c, 2.4, SILL, HEAD) for c in north_win])
for i, c in enumerate(north_win):
    window_glazing(f"win_n{i}", 'x', WALL_OUT, c, 2.4)

# South wall: entrance + 2 windows
wall_with_openings("wall_south", 'x', -WALL_OUT, (-R - WALL_T, R + WALL_T),
                   [(-5.0, 2.4, SILL, HEAD), (0.0, 2.0, 0.0, 2.2), (5.0, 2.4, SILL, HEAD)])
for i, c in enumerate([-5.0, 5.0]):
    window_glazing(f"win_s{i}", 'x', -WALL_OUT, c, 2.4)
# entrance glazed doors
add_box("entrance_frame_t", (2.0, WALL_T, 0.08), (0, -WALL_OUT, 2.16), mat=M['alu'], coll="Shell")
for s, x in ((-1, -0.5), (1, 0.5)):
    add_box(f"entrance_door_{'l' if s < 0 else 'r'}", (0.94, 0.05, 2.1), (x, -WALL_OUT, 1.05),
            mat=M['glass'], coll="Shell")
    add_box(f"entrance_stile_{'l' if s < 0 else 'r'}", (0.05, 0.06, 2.1), (x + s * 0.46, -WALL_OUT, 1.05),
            mat=M['alu'], coll="Shell")
    add_cyl(f"entrance_handle_{'l' if s < 0 else 'r'}", 0.018, 0.9,
            (x + s * 0.36, -WALL_OUT - 0.06, 1.05), rot=(0, 0, 0), mat=M['alu'], coll="Shell")

# West wall: solid
wall_with_openings("wall_west", 'y', -WALL_OUT, (-R - WALL_T, R + WALL_T), [])

# Skirting boards around the perimeter
for nm, size, loc in [
    ("skirt_n", (2 * R, 0.02, 0.09), (0, R - 0.01, 0.045)),
    ("skirt_s", (2 * R, 0.02, 0.09), (0, -R + 0.01, 0.045)),
    ("skirt_e", (0.02, 2 * R, 0.09), (R - 0.01, 0, 0.045)),
    ("skirt_w", (0.02, 2 * R, 0.09), (-R + 0.01, 0, 0.045)),
]:
    add_box(nm, size, loc, mat=M['skirting'], coll="Shell")

# ---- interior partitions -------------------------------------------------
# Conference room (NE): partitions at y=3.0 (x 3.0..R) and x=3.0 (y 3.0..R)
wall_with_openings("part_conf_s", 'x', 3.0, (3.0, R), [(3.7, 1.0, 0.0, 2.1)])
# west side: solid base + glazed upper band (office glass partition)
add_box("part_conf_w_base", (WALL_T, R - 3.0, 1.0), (3.0, (3.0 + R) / 2, 0.5), mat=M['wall'], coll="Shell")
add_box("part_conf_w_top", (WALL_T, R - 3.0, CEIL_H - 2.6), (3.0, (3.0 + R) / 2, (2.6 + CEIL_H) / 2),
        mat=M['wall'], coll="Shell")
add_box("part_conf_w_glass", (0.014, R - 3.0, 1.6), (3.0, (3.0 + R) / 2, 1.8), mat=M['glass'], coll="Shell")
for i in range(1, 5):
    y = 3.0 + i * (R - 3.0) / 5
    add_box(f"part_conf_mul{i}", (0.05, 0.05, 1.6), (3.0, y, 1.8), mat=M['alu'], coll="Shell")
add_box("part_conf_rail_b", (0.07, R - 3.0, 0.05), (3.0, (3.0 + R) / 2, 1.02), mat=M['alu'], coll="Shell")
add_box("part_conf_rail_t", (0.07, R - 3.0, 0.05), (3.0, (3.0 + R) / 2, 2.58), mat=M['alu'], coll="Shell")

# Private office (NW): partitions at y=4.6 (x -R..-4.4) and x=-4.4 (y 4.6..R)
wall_with_openings("part_off_s", 'x', 4.6, (-R, -4.4), [(-5.4, 1.0, 0.0, 2.1)])
wall_with_openings("part_off_e", 'y', -4.4, (4.6, R), [])

# door leaves (ajar) + frames
def doorway(name, cx, cy, width, facing, swing):
    add_box(f"{name}_frame_l", (0.07, 0.24, 2.12), (cx - width / 2, cy, 1.06), mat=M['wood_light'], coll="Shell")
    add_box(f"{name}_frame_r", (0.07, 0.24, 2.12), (cx + width / 2, cy, 1.06), mat=M['wood_light'], coll="Shell")
    add_box(f"{name}_frame_t", (width + 0.14, 0.24, 0.07), (cx, cy, 2.12), mat=M['wood_light'], coll="Shell")
    hinge_x = cx - width / 2 + 0.03
    leaf = add_box(f"{name}_leaf", (width, 0.04, 2.05), (0, 0, 0), mat=M['wood_light'], coll="Shell")
    ang = math.radians(swing)
    leaf.location = (hinge_x + width / 2 * math.cos(ang), cy + facing * width / 2 * math.sin(ang), 1.03)
    leaf.rotation_euler = (0, 0, facing * ang)
    bevel(leaf, 0.004)
    add_cyl(f"{name}_handle", 0.022, 0.12,
            (leaf.location.x + width * 0.36 * math.cos(ang), leaf.location.y + facing * width * 0.36 * math.sin(ang), 1.05),
            rot=(math.pi / 2, 0, facing * ang), mat=M['alu'], coll="Shell")

doorway("door_conf", 3.7, 3.0, 1.0, facing=-1, swing=72)
doorway("door_office", -5.4, 4.6, 1.0, facing=-1, swing=65)

# freestanding accent backdrop behind the reception counter
add_box("reception_backdrop", (3.4, 0.10, 1.95), (0, -6.85, 0.975), mat=M['wall_accent'], coll="Shell")
add_box("reception_backdrop_trim", (3.4, 0.12, 0.05), (0, -6.85, 1.93), mat=M['alu'], coll="Shell")

# floor finishes
add_plane("floor_vinyl_break", (7.1, 5.2), (-0.75, 7.35, 0.004), mat=M['vinyl'])
add_plane("rug_lounge", (3.6, 3.0), (6.4, -1.6, 0.004), mat=M['carpet_lounge'])


# ---------------------------------------------------------------------------
# Lighting
# ---------------------------------------------------------------------------

def ceiling_panel(name, x, y):
    add_box(f"{name}_housing", (1.24, 0.34, 0.06), (x, y, CEIL_H - 0.03), mat=M['alu'], coll="Lighting")
    add_box(f"{name}_diffuser", (1.18, 0.28, 0.03), (x, y, CEIL_H - 0.045), mat=M['panel_light'], coll="Lighting")
    ld = bpy.data.lights.new(f"{name}_lamp", type='AREA')
    ld.shape = 'RECTANGLE'
    ld.size, ld.size_y = 1.18, 0.28
    ld.energy = 42.0
    ld.color = (1.0, 0.90, 0.78)
    lo = bpy.data.objects.new(f"{name}_lamp", ld)
    lo.location = (x, y, CEIL_H - 0.07)
    link(lo, "Lighting")

for gx in (-7.5, -2.5, 2.5, 7.5):
    for gy in (-8.0, -4.0, 0.0, 4.0, 8.0):
        ceiling_panel(f"panel_{gx}_{gy}".replace("-", "m").replace(".", ""), gx, gy)

sun_d = bpy.data.lights.new("sun", type='SUN')
sun_d.energy = 1.35
sun_d.angle = math.radians(2.0)
sun_d.color = (1.0, 0.95, 0.87)
sun = bpy.data.objects.new("sun", sun_d)
sun.location = (14, 6, 12)
sun.rotation_euler = (0, math.radians(52), math.radians(22))
link(sun, "Lighting")

world = bpy.data.worlds.new("World")
scene.world = world
world.use_nodes = True
wnt = world.node_tree
bg = wnt.nodes["Background"]
sky = wnt.nodes.new("ShaderNodeTexSky")
sky_types = [e.identifier for e in sky.bl_rna.properties['sky_type'].enum_items]
sky.sky_type = 'MULTIPLE_SCATTERING' if 'MULTIPLE_SCATTERING' in sky_types else sky_types[0]
for attr, val in [("sun_elevation", math.radians(38)), ("sun_rotation", math.radians(75)),
                  ("sun_disc", False), ("altitude", 120)]:
    if hasattr(sky, attr):
        try:
            setattr(sky, attr, val)
        except Exception:
            pass
wnt.links.new(sky.outputs["Color"], bg.inputs["Color"])
bg.inputs["Strength"].default_value = 0.22


# ---------------------------------------------------------------------------
# Exterior context (visible through the windows)
# ---------------------------------------------------------------------------

add_plane("exterior_ground", (300, 300), (0, 0, -0.06), mat=M['concrete'], coll="Exterior")
for i, (bx, by, bw, bd, bh) in enumerate([
        (46, 14, 16, 14, 26), (38, -22, 12, 12, 17), (16, 44, 18, 16, 31),
        (-24, 40, 14, 12, 21), (60, -4, 20, 18, 38)]):
    add_box(f"ext_building_{i}", (bw, bd, bh), (bx, by, bh / 2 - 0.05), mat=M['building'], coll="Exterior")


# ---------------------------------------------------------------------------
# Furniture builders
# ---------------------------------------------------------------------------

def make_desk(name, cx, cy, rot=0.0, w=1.6, d=0.8, h=0.74, top_mat=None):
    top_mat = top_mat or M['wood_desk']
    parts = []
    t = add_box(f"{name}_top", (w, d, 0.038), L(cx, cy, rot, 0, 0, h - 0.019), (0, 0, rot), top_mat)
    parts.append(bevel(t, 0.005))
    for sx in (-1, 1):
        for sy in (-1, 1):
            lx, ly = sx * (w / 2 - 0.07), sy * (d / 2 - 0.07)
            leg = add_box(f"{name}_leg", (0.05, 0.05, h - 0.04), L(cx, cy, rot, lx, ly, (h - 0.04) / 2),
                          (0, 0, rot), M['metal'])
            parts.append(leg)
        rail = add_box(f"{name}_rail", (0.04, d - 0.14, 0.04), L(cx, cy, rot, sx * (w / 2 - 0.07), 0, h - 0.10),
                       (0, 0, rot), M['metal'])
        parts.append(rail)
    modesty = add_box(f"{name}_modesty", (w - 0.2, 0.02, 0.34), L(cx, cy, rot, 0, d / 2 - 0.08, 0.44),
                      (0, 0, rot), M['metal'])
    parts.append(modesty)
    return parts, h


def make_monitor(name, cx, cy, rot, top_h=0.74, w=0.56, hgt=0.34):
    parts = []
    parts.append(add_cyl(f"{name}_base", 0.11, 0.02, L(cx, cy, rot, 0, 0, top_h + 0.01), mat=M['plastic_black']))
    parts.append(add_box(f"{name}_neck", (0.05, 0.04, 0.2), L(cx, cy, rot, 0, 0, top_h + 0.12), (0, 0, rot), M['plastic_black']))
    panel = add_box(f"{name}_panel", (w, 0.022, hgt), L(cx, cy, rot, 0, 0, top_h + 0.22 + hgt / 2), (0, 0, rot), M['plastic_black'])
    parts.append(bevel(panel, 0.004))
    parts.append(add_box(f"{name}_screen", (w - 0.03, 0.006, hgt - 0.035),
                         L(cx, cy, rot, 0, -0.014, top_h + 0.22 + hgt / 2), (0, 0, rot), M['screen']))
    return parts


def make_office_chair(name, cx, cy, rot):
    parts = []
    sh = 0.46
    seat = add_box(f"{name}_seat", (0.48, 0.47, 0.09), L(cx, cy, rot, 0, 0, sh), (0, 0, rot), M['fabric_blue'])
    parts.append(bevel(seat, 0.02, 3))
    back = add_box(f"{name}_back", (0.45, 0.07, 0.52), L(cx, cy, rot, 0, 0.235, sh + 0.33),
                   (math.radians(-8), 0, rot), M['fabric_blue'])
    parts.append(bevel(back, 0.02, 3))
    parts.append(add_box(f"{name}_backstem", (0.07, 0.06, 0.18), L(cx, cy, rot, 0, 0.22, sh + 0.06), (0, 0, rot), M['plastic_black']))
    for s in (-1, 1):
        parts.append(add_box(f"{name}_arm", (0.05, 0.32, 0.04), L(cx, cy, rot, s * 0.26, 0.02, sh + 0.21), (0, 0, rot), M['plastic_black']))
        parts.append(add_box(f"{name}_armpost", (0.04, 0.05, 0.16), L(cx, cy, rot, s * 0.26, 0.1, sh + 0.12), (0, 0, rot), M['plastic_black']))
    parts.append(add_cyl(f"{name}_column", 0.035, 0.28, L(cx, cy, rot, 0, 0, sh - 0.19), mat=M['metal'], verts=16))
    parts.append(add_cyl(f"{name}_gas", 0.055, 0.1, L(cx, cy, rot, 0, 0, sh - 0.30), mat=M['plastic_black'], verts=16))
    for i in range(5):
        a = rot + i * (2 * math.pi / 5)
        parts.append(add_box(f"{name}_spoke{i}", (0.30, 0.045, 0.035),
                             (cx + 0.15 * math.cos(a), cy + 0.15 * math.sin(a), 0.07), (0, 0, a), M['plastic_black']))
        parts.append(add_cyl(f"{name}_caster{i}", 0.028, 0.022,
                             (cx + 0.29 * math.cos(a), cy + 0.29 * math.sin(a), 0.028),
                             rot=(math.pi / 2, 0, a), mat=M['plastic_black'], verts=12))
    return parts


def make_desk_clutter(name, cx, cy, rot, top_h):
    parts = []
    kb = add_box(f"{name}_keyboard", (0.43, 0.14, 0.018), L(cx, cy, rot, -0.05, -0.22, top_h + 0.009), (0, 0, rot), M['plastic_black'])
    parts.append(bevel(kb, 0.003))
    parts.append(add_sphere(f"{name}_mouse", 0.04, L(cx, cy, rot, 0.33, -0.22, top_h + 0.018), scale=(1.0, 1.4, 0.42), rot=(0, 0, rot), mat=M['plastic_black']))
    if random.random() < 0.7:
        parts.append(add_cyl(f"{name}_mug", 0.042, 0.1, L(cx, cy, rot, -0.55, -0.12, top_h + 0.05),
                             mat=random.choice([M['plastic_white'], M['fabric_teal'], M['nav_red']]), verts=20, smooth=True))
    if random.random() < 0.6:
        pap = add_box(f"{name}_papers", (0.22, 0.30, 0.012), L(cx, cy, rot, 0.5, 0.05, top_h + 0.006),
                      (0, 0, rot + random.uniform(-0.3, 0.3)), M['paper'])
        parts.append(pap)
    return parts


def make_desk_pod(pod, cx, cy, rot=0.0):
    """Four desks back-to-back around a divider.

    Desk-local convention: the user sits at local -y and faces +y, so the row on
    the +y side of the divider is yawed by pi to seat its users on the outside.
    """
    made = []
    desk_info = []
    for i, (dx, dy, flip) in enumerate([(-0.85, 0.45, math.pi), (0.85, 0.45, math.pi),
                                        (-0.85, -0.45, 0.0), (0.85, -0.45, 0.0)]):
        wx, wy, _ = L(cx, cy, rot, dx, dy, 0)
        drot = rot + flip
        parts, h = make_desk(f"{pod}_desk{i}", wx, wy, drot)
        parts += make_monitor(f"{pod}_mon{i}", *L(wx, wy, drot, 0, 0.26, 0)[:2], drot, h)
        parts += make_desk_clutter(f"{pod}_clut{i}", wx, wy, drot, h)
        chair_x, chair_y, _ = L(wx, wy, drot, 0, -0.78, 0)
        parts += make_office_chair(f"{pod}_chair{i}", chair_x, chair_y, drot + math.pi + random.uniform(-0.45, 0.45))
        emp = group(f"{pod}_workstation{i}", parts, (wx, wy, 0))
        made.append(emp)
        desk_info.append((f"{pod}_workstation{i}", wx, wy, drot, h))
        landmark(f"{pod}_workstation{i}", f"desk / workstation {i + 1}", "workstation",
                 (wx, wy, 0.0), (1.6, 0.8, 0.74), zone="open_plan")
    # shared divider panel
    div = add_box(f"{pod}_divider", (3.5, 0.05, 0.42), L(cx, cy, rot, 0, 0, 0.74 + 0.21), (0, 0, rot), M['fabric_teal'])
    bevel(div, 0.01)
    return desk_info


def make_cabinet(name, cx, cy, rot, w=0.8, d=0.45, h=1.05, drawers=3, mat=None):
    mat = mat or M['metal']
    parts = [add_box(f"{name}_body", (w, d, h), L(cx, cy, rot, 0, 0, h / 2), (0, 0, rot), mat)]
    bevel(parts[0], 0.006)
    for i in range(drawers):
        z = h / drawers * (i + 0.5)
        parts.append(add_box(f"{name}_face{i}", (w - 0.05, 0.02, h / drawers - 0.03),
                             L(cx, cy, rot, 0, -d / 2 - 0.005, z), (0, 0, rot), mat))
        parts.append(add_box(f"{name}_pull{i}", (0.22, 0.025, 0.02),
                             L(cx, cy, rot, 0, -d / 2 - 0.025, z + h / drawers * 0.28), (0, 0, rot), M['alu']))
    return parts


def make_plant(name, cx, cy, scale=1.0, coll="Furniture"):
    """Potted shrub: dense cluster of small leaf clumps in an ellipsoid crown."""
    parts = []
    ph = 0.34 * scale
    parts.append(add_cyl(f"{name}_pot", 0.24 * scale, ph, (cx, cy, ph / 2), mat=M['pot'], verts=24, coll=coll))
    parts.append(add_cyl(f"{name}_soil", 0.215 * scale, 0.02, (cx, cy, ph - 0.005), mat=M['soil'], verts=24, coll=coll))
    crown_z = ph + 0.42 * scale
    for i in range(4):
        a = i * math.pi / 2 + random.uniform(-0.3, 0.3)
        parts.append(add_cyl(f"{name}_stem{i}", 0.016 * scale, 0.55 * scale,
                             (cx + 0.05 * scale * math.cos(a), cy + 0.05 * scale * math.sin(a), ph + 0.22 * scale),
                             rot=(random.uniform(-0.18, 0.18), random.uniform(-0.18, 0.18), a),
                             mat=M['soil'], verts=8, coll=coll))
    for i in range(18):
        a = random.uniform(0, 2 * math.pi)
        r = random.uniform(0.0, 0.30) * scale
        z = crown_z + random.uniform(-0.16, 0.34) * scale
        parts.append(add_sphere(f"{name}_leaf{i}", random.uniform(0.085, 0.155) * scale,
                                (cx + r * math.cos(a), cy + r * math.sin(a), z),
                                scale=(1.35, 1.1, 0.66),
                                rot=(random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), a),
                                mat=M['leaf'], coll=coll))
    return parts


def make_sofa(name, cx, cy, rot, w=2.1, d=0.88):
    parts = []
    parts.append(bevel(add_box(f"{name}_base", (w, d, 0.34), L(cx, cy, rot, 0, 0, 0.17), (0, 0, rot), M['fabric_grey']), 0.03, 3))
    parts.append(bevel(add_box(f"{name}_seat", (w - 0.3, d - 0.16, 0.14), L(cx, cy, rot, 0, -0.04, 0.41), (0, 0, rot), M['fabric_grey']), 0.04, 3))
    parts.append(bevel(add_box(f"{name}_back", (w, 0.2, 0.52), L(cx, cy, rot, 0, d / 2 - 0.1, 0.6), (0, 0, rot), M['fabric_grey']), 0.04, 3))
    seats = (-1, 1) if w > 1.3 else (0,)
    for i, sx in enumerate(seats):
        cw = (w / 2 - 0.24) if len(seats) > 1 else (w - 0.42)
        parts.append(bevel(add_box(f"{name}_cushion{i}", (cw, 0.14, 0.42),
                                   L(cx, cy, rot, sx * (w / 4 - 0.03), d / 2 - 0.24, 0.62),
                                   (math.radians(6), 0, rot), M['fabric_grey']), 0.05, 3))
        parts.append(bevel(add_box(f"{name}_seatpad{i}", (cw + 0.02, d - 0.30, 0.10),
                                   L(cx, cy, rot, sx * (w / 4 - 0.02), -0.06, 0.49),
                                   (0, 0, rot), M['fabric_grey']), 0.04, 3))
    for s in (-1, 1):
        parts.append(bevel(add_box(f"{name}_arm", (0.17, d, 0.24), L(cx, cy, rot, s * (w / 2 - 0.085), 0, 0.46), (0, 0, rot), M['fabric_grey']), 0.04, 3))
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(add_cyl(f"{name}_foot", 0.03, 0.08, L(cx, cy, rot, sx * (w / 2 - 0.16), sy * (d / 2 - 0.12), 0.04), mat=M['metal_dark'], verts=10))
    return parts


def make_coffee_table(name, cx, cy, rot, w=1.1, d=0.6, h=0.42):
    parts = [bevel(add_box(f"{name}_top", (w, d, 0.04), L(cx, cy, rot, 0, 0, h - 0.02), (0, 0, rot), M['wood_dark']), 0.006)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(add_box(f"{name}_leg", (0.045, 0.045, h - 0.04),
                                 L(cx, cy, rot, sx * (w / 2 - 0.08), sy * (d / 2 - 0.07), (h - 0.04) / 2), (0, 0, rot), M['metal_dark']))
    return parts, h


def make_conference_table(name, cx, cy, rot, w=3.4, d=1.4, h=0.75):
    parts = [bevel(add_box(f"{name}_top", (w, d, 0.05), L(cx, cy, rot, 0, 0, h - 0.025), (0, 0, rot), M['wood_dark']), 0.008)]
    for s in (-1, 1):
        parts.append(add_box(f"{name}_pedestal", (0.12, d - 0.5, h - 0.09), L(cx, cy, rot, s * (w / 2 - 0.6), 0, (h - 0.09) / 2), (0, 0, rot), M['metal']))
        parts.append(add_box(f"{name}_foot", (0.6, d - 0.35, 0.05), L(cx, cy, rot, s * (w / 2 - 0.6), 0, 0.025), (0, 0, rot), M['metal_dark']))
    parts.append(add_box(f"{name}_beam", (w - 1.4, 0.1, 0.1), L(cx, cy, rot, 0, 0, h - 0.3), (0, 0, rot), M['metal']))
    return parts, h


def make_reception_desk(name, cx, cy, rot):
    parts = []
    parts.append(bevel(add_box(f"{name}_counter", (2.6, 0.75, 0.72), L(cx, cy, rot, 0, 0, 0.36), (0, 0, rot), M['wood_light']), 0.008))
    parts.append(bevel(add_box(f"{name}_return", (0.75, 1.5, 0.72), L(cx, cy, rot, -1.68, 0.72, 0.36), (0, 0, rot), M['wood_light']), 0.008))
    parts.append(bevel(add_box(f"{name}_transaction", (2.9, 0.28, 0.22), L(cx, cy, rot, 0, -0.12, 1.0), (0, 0, rot), M['wood_dark']), 0.01))
    parts.append(add_box(f"{name}_apron", (2.9, 0.05, 0.30), L(cx, cy, rot, 0, -0.36, 0.92), (0, 0, rot), M['fabric_teal']))
    return parts, 0.72


def make_kitchen_counter(name, cx, cy, rot, w=3.0):
    parts = []
    parts.append(bevel(add_box(f"{name}_base", (w, 0.62, 0.86), L(cx, cy, rot, 0, 0, 0.43), (0, 0, rot), M['plastic_white']), 0.006))
    parts.append(bevel(add_box(f"{name}_top", (w + 0.06, 0.66, 0.045), L(cx, cy, rot, 0, 0, 0.88), (0, 0, rot), M['metal']), 0.006))
    parts.append(add_box(f"{name}_splash", (w + 0.06, 0.02, 0.5), L(cx, cy, rot, 0, 0.33, 1.15), (0, 0, rot), M['vinyl']))
    for i in range(int(w // 0.75)):
        x = -w / 2 + 0.375 + i * 0.75
        parts.append(add_box(f"{name}_door{i}", (0.72, 0.02, 0.78), L(cx, cy, rot, x, -0.32, 0.44), (0, 0, rot), M['plastic_white']))
        parts.append(add_box(f"{name}_pull{i}", (0.4, 0.02, 0.02), L(cx, cy, rot, x, -0.345, 0.78), (0, 0, rot), M['alu']))
    # upper cupboards
    parts.append(bevel(add_box(f"{name}_upper", (w, 0.36, 0.7), L(cx, cy, rot, 0, 0.14, 1.95), (0, 0, rot), M['plastic_white']), 0.006))
    # sink recess + tap
    parts.append(add_box(f"{name}_sink", (0.5, 0.4, 0.02), L(cx, cy, rot, w / 2 - 0.7, 0, 0.89), (0, 0, rot), M['metal']))
    parts.append(add_cyl(f"{name}_tap", 0.018, 0.28, L(cx, cy, rot, w / 2 - 0.7, 0.22, 1.04), mat=M['alu'], verts=12))
    return parts


def make_whiteboard(name, cx, cy, rot, w=2.4, h=1.2, z=1.5):
    parts = [add_box(f"{name}_frame", (w + 0.06, 0.04, h + 0.06), L(cx, cy, rot, 0, 0, z), (0, 0, rot), M['alu']),
             add_box(f"{name}_surface", (w, 0.02, h), L(cx, cy, rot, 0, -0.02, z), (0, 0, rot), M['whiteboard']),
             add_box(f"{name}_tray", (w * 0.6, 0.07, 0.03), L(cx, cy, rot, 0, -0.05, z - h / 2 - 0.05), (0, 0, rot), M['alu'])]
    return parts


def make_wall_screen(name, cx, cy, rot, w=1.6, h=0.92, z=1.7):
    parts = [bevel(add_box(f"{name}_body", (w, 0.06, h), L(cx, cy, rot, 0, 0, z), (0, 0, rot), M['plastic_black']), 0.006),
             add_box(f"{name}_screen", (w - 0.04, 0.01, h - 0.04), L(cx, cy, rot, 0, -0.035, z), (0, 0, rot), M['screen'])]
    return parts


def make_water_cooler(name, cx, cy, rot):
    parts = [bevel(add_box(f"{name}_body", (0.34, 0.34, 1.0), (cx, cy, 0.5), (0, 0, rot), M['plastic_white']), 0.01),
             add_cyl(f"{name}_bottle", 0.17, 0.46, (cx, cy, 1.23), mat=M['glass'], verts=20, smooth=True),
             add_box(f"{name}_tap", (0.12, 0.08, 0.12), L(cx, cy, rot, 0, -0.2, 0.76), (0, 0, rot), M['plastic_black'])]
    return parts


def make_printer(name, cx, cy, rot):
    parts = [bevel(add_box(f"{name}_body", (0.62, 0.55, 0.72), (cx, cy, 0.36), (0, 0, rot), M['plastic_white']), 0.01),
             add_box(f"{name}_top", (0.62, 0.55, 0.09), (cx, cy, 0.76), (0, 0, rot), M['plastic_black']),
             add_box(f"{name}_tray", (0.5, 0.1, 0.02), L(cx, cy, rot, 0, -0.3, 0.55), (0, 0, rot), M['plastic_black']),
             add_box(f"{name}_panel", (0.2, 0.12, 0.02), L(cx, cy, rot, 0.18, -0.2, 0.81), (math.radians(-25), 0, rot), M['screen'])]
    return parts


def make_bin(name, cx, cy):
    return [add_cyl(f"{name}_body", 0.17, 0.42, (cx, cy, 0.21), mat=M['metal'], verts=20),
            add_cyl(f"{name}_rim", 0.18, 0.03, (cx, cy, 0.42), mat=M['metal_dark'], verts=20)]


def make_round_table(name, cx, cy, r=0.6, h=0.74):
    parts = [add_cyl(f"{name}_top", r, 0.04, (cx, cy, h - 0.02), mat=M['wood_light'], verts=40),
             add_cyl(f"{name}_column", 0.06, h - 0.06, (cx, cy, (h - 0.06) / 2), mat=M['metal'], verts=20),
             add_cyl(f"{name}_foot", 0.34, 0.03, (cx, cy, 0.015), mat=M['metal_dark'], verts=32)]
    bevel(parts[0], 0.005)
    return parts, h


def make_clock(name, cx, cy, rot, z=2.3):
    parts = [add_cyl(f"{name}_body", 0.17, 0.05, L(cx, cy, rot, 0, 0, z), rot=(math.pi / 2, 0, rot), mat=M['plastic_white'], verts=32),
             add_box(f"{name}_hand_h", (0.015, 0.09, 0.008), L(cx, cy, rot, 0, -0.03, z + 0.03), (math.pi / 2, 0, rot), M['plastic_black']),
             add_box(f"{name}_hand_m", (0.012, 0.14, 0.008), L(cx, cy, rot, 0.04, -0.03, z - 0.03), (math.pi / 2, 0, rot + 0.9), M['plastic_black'])]
    return parts


def make_art(name, cx, cy, rot, w=1.0, h=0.7, z=1.8, color=(0.2, 0.35, 0.5)):
    art_mat = marble_mat(f"{name}_canvas", color, tuple(min(1, c * 1.9 + 0.15) for c in color))
    return [add_box(f"{name}_frame", (w + 0.05, 0.04, h + 0.05), L(cx, cy, rot, 0, 0, z), (0, 0, rot), M['wood_dark']),
            add_box(f"{name}_canvas", (w, 0.02, h), L(cx, cy, rot, 0, -0.02, z), (0, 0, rot), art_mat)]


# ---------------------------------------------------------------------------
# Zone layout
# ---------------------------------------------------------------------------

# ---- open plan desk pods ----
podA = make_desk_pod("podA", -6.5, 1.2, rot=0.0)
podB = make_desk_pod("podB", -6.5, -3.6, rot=0.0)
podC = make_desk_pod("podC", -1.0, -1.2, rot=math.radians(90))

# ---- reception near the entrance ----
rec_parts, rec_h = make_reception_desk("reception", 0.0, -8.0, math.radians(180))
group("reception_desk", rec_parts, (0, -8.0, 0))
landmark("reception_desk", "reception desk", "furniture", (0, -8.0, 0), (2.9, 1.5, 1.05), zone="reception")
make_office_chair("reception_chair", 0.0, -7.15, 0.0)   # chair faces -y by default
group("reception_plant", make_plant("reception_plant", 2.6, -8.9, 1.05), (2.6, -8.9, 0))
landmark("reception_plant", "potted plant by reception", "plant", (2.6, -8.9, 0), (0.9, 0.9, 1.6), zone="reception")
for i, wx in enumerate((-2.7, -3.6)):
    make_office_chair(f"waiting_chair{i}", wx, -8.9, math.radians(165 + i * 25))

# ---- lounge (east) ----
group("lounge_sofa", make_sofa("lounge_sofa", 7.6, -1.6, math.radians(-90)), (7.6, -1.6, 0))
landmark("lounge_sofa", "grey sofa in the lounge", "furniture", (7.6, -1.6, 0), (2.1, 0.88, 0.85), zone="lounge")
ct_parts, ct_h = make_coffee_table("lounge_table", 6.0, -1.6, 0.0)
group("lounge_coffee_table", ct_parts, (6.0, -1.6, 0))
landmark("lounge_coffee_table", "coffee table in the lounge", "furniture", (6.0, -1.6, 0), (1.1, 0.6, 0.42), zone="lounge")
group("lounge_plant", make_plant("lounge_plant", 8.9, 0.9, 1.15), (8.9, 0.9, 0))
landmark("lounge_plant", "large potted plant in the lounge", "plant", (8.9, 0.9, 0), (1.0, 1.0, 1.8), zone="lounge")

# ---- printer / storage strip along the west wall ----
group("printer_station", make_printer("printer", -9.45, -7.6, math.radians(90)), (-9.45, -7.6, 0))
landmark("printer_station", "office printer", "appliance", (-9.45, -7.6, 0), (0.62, 0.55, 0.8), zone="open_plan")
for i, y in enumerate((-6.2, -5.35, -4.5)):
    group(f"filing_cabinet_{i}", make_cabinet(f"cabinet{i}", -9.55, y, math.radians(90), w=0.8, d=0.45, h=1.05),
          (-9.55, y, 0))
landmark("filing_cabinet_1", "filing cabinets", "furniture", (-9.55, -5.35, 0), (0.45, 2.4, 1.05), zone="open_plan")
group("bin_open_plan", make_bin("bin_openplan", -9.3, -8.6), (-9.3, -8.6, 0))
group("clock_west", make_clock("clock", -9.92, -1.0, math.radians(90), z=2.35), (-9.92, -1.0, 2.35))
landmark("clock_west", "wall clock", "decor", (-9.92, -1.0, 2.35), (0.34, 0.05, 0.34), zone="open_plan")
group("art_south", make_art("art_south", -2.0, -9.92, 0.0, 1.2, 0.8, 1.85, (0.18, 0.32, 0.45)), (-2.0, -9.92, 1.85))
group("art_west", make_art("art_west", -9.92, 3.0, math.radians(90), 0.9, 1.2, 1.8, (0.42, 0.22, 0.12)), (-9.92, 3.0, 1.8))

# ---- standing collaboration point in the open centre ----
collab = [add_cyl("collab_top", 0.52, 0.045, (2.6, -4.7, 1.03), mat=M['wood_light'], verts=40),
          add_cyl("collab_column", 0.055, 1.0, (2.6, -4.7, 0.51), mat=M['metal'], verts=20),
          add_cyl("collab_foot", 0.34, 0.03, (2.6, -4.7, 0.015), mat=M['metal_dark'], verts=32)]
group("collab_table", collab, (2.6, -4.7, 0))
landmark("collab_table", "standing collaboration table", "furniture", (2.6, -4.7, 0), (1.04, 1.04, 1.05), zone="open_plan")
for i in range(3):
    a = math.radians(30 + i * 120)
    sx, sy = 2.6 + 0.85 * math.cos(a), -4.7 + 0.85 * math.sin(a)
    group(f"stool_{i}", [
        add_cyl(f"stool{i}_seat", 0.18, 0.07, (sx, sy, 0.72), mat=M['fabric_teal'], verts=24),
        add_cyl(f"stool{i}_column", 0.03, 0.68, (sx, sy, 0.34), mat=M['metal'], verts=14),
        add_cyl(f"stool{i}_foot", 0.22, 0.025, (sx, sy, 0.012), mat=M['metal_dark'], verts=24),
        add_cyl(f"stool{i}_ring", 0.16, 0.02, (sx, sy, 0.22), mat=M['metal'], verts=24)], (sx, sy, 0))

# ---- conference room (NE) ----
conf_parts, conf_h = make_conference_table("conf_table", 6.5, 6.4, 0.0, w=3.4, d=1.4)
group("conference_table", conf_parts, (6.5, 6.4, 0))
landmark("conference_table", "conference table", "furniture", (6.5, 6.4, 0), (3.4, 1.4, 0.75), zone="conference_room")
for i, x in enumerate((-1.15, 0.0, 1.15)):
    make_office_chair(f"conf_chair_n{i}", 6.5 + x, 7.6, 0.0)                 # faces south
    make_office_chair(f"conf_chair_s{i}", 6.5 + x, 5.2, math.radians(180))   # faces north
make_office_chair("conf_chair_e", 8.75, 6.4, math.radians(-90))              # faces west
make_office_chair("conf_chair_w", 4.25, 6.4, math.radians(90))               # faces east
group("whiteboard", make_whiteboard("whiteboard", 5.4, 9.92, 0.0), (5.4, 9.92, 1.5))
landmark("whiteboard", "whiteboard", "decor", (5.4, 9.92, 1.5), (2.4, 0.06, 1.2), zone="conference_room")
group("conference_screen", make_wall_screen("conf_screen", 8.6, 9.92, 0.0), (8.6, 9.92, 1.7))
landmark("conference_screen", "wall-mounted display screen", "appliance", (8.6, 9.92, 1.7), (1.6, 0.06, 0.92), zone="conference_room")
group("conference_credenza", make_cabinet("credenza", 3.6, 8.6, math.radians(90), w=1.6, d=0.45, h=0.75, drawers=2, mat=M['wood_light']),
      (3.6, 8.6, 0))
landmark("conference_credenza", "credenza cabinet", "furniture", (3.6, 8.6, 0), (0.45, 1.6, 0.75), zone="conference_room")
group("conference_plant", make_plant("conf_plant", 9.2, 3.9, 1.0), (9.2, 3.9, 0))

# ---- private office (NW) ----
# manager sits on the north side facing south -> desk yawed by pi
po_rot = math.pi
po_parts, po_h = make_desk("po_desk", -7.4, 7.6, po_rot, w=1.8, d=0.9)
po_parts += make_monitor("po_mon", *L(-7.4, 7.6, po_rot, 0, 0.28, 0)[:2], po_rot, po_h)
po_parts += make_desk_clutter("po_clut", -7.4, 7.6, po_rot, po_h)
group("private_office_desk", po_parts, (-7.4, 7.6, 0))
landmark("private_office_desk", "manager's desk", "workstation", (-7.4, 7.6, 0), (1.8, 0.9, 0.74), zone="private_office")
make_office_chair("po_chair", -7.4, 8.45, 0.0)
for i, x in enumerate((-8.1, -6.7)):
    make_office_chair(f"po_guest{i}", x, 6.5, math.radians(180 + (i - 0.5) * 30))
group("private_office_plant", make_plant("po_plant", -9.2, 9.1, 0.95), (-9.2, 9.1, 0))
group("private_office_bin", make_bin("po_bin", -8.8, 6.4), (-8.8, 6.4, 0))

# ---- break area (N centre) ----
group("kitchen_counter", make_kitchen_counter("kitchen", -1.9, 9.6, 0.0, w=3.0), (-1.9, 9.6, 0))
landmark("kitchen_counter", "kitchen counter / kitchenette", "furniture", (-1.9, 9.6, 0), (3.0, 0.66, 0.9), zone="break_area")
group("water_cooler", make_water_cooler("cooler", -4.0, 9.4, 0.0), (-4.0, 9.4, 0))
landmark("water_cooler", "water cooler", "appliance", (-4.0, 9.4, 0), (0.34, 0.34, 1.45), zone="break_area")
rt_parts, rt_h = make_round_table("break_table", 0.6, 6.6)
group("break_table", rt_parts, (0.6, 6.6, 0))
landmark("break_table", "round break-room table", "furniture", (0.6, 6.6, 0), (1.2, 1.2, 0.74), zone="break_area")
# coffee machine + mugs on the counter
group("coffee_machine", [
    bevel(add_box("coffee_body", (0.3, 0.34, 0.42), (-3.0, 9.6, 1.11), (0, 0, 0), M['plastic_black']), 0.008),
    add_box("coffee_spout", (0.14, 0.12, 0.1), (-3.0, 9.44, 0.97), (0, 0, 0), M['metal']),
    add_cyl("coffee_pot", 0.07, 0.16, (-3.0, 9.44, 0.98), mat=M['glass'], verts=20, smooth=True)],
    (-3.0, 9.6, 0.9))
landmark("coffee_machine", "coffee machine", "appliance", (-3.0, 9.6, 0.9), (0.3, 0.34, 0.42), zone="break_area")
for i, x in enumerate((-1.0, -0.75)):
    add_cyl(f"break_mug{i}", 0.04, 0.1, (x, 9.55, 0.95), mat=M['plastic_white'], verts=18, smooth=True)
# lockers + noticeboard on the blank partition wall of the break area
locker_parts = []
for i in range(4):
    y = 5.6 + i * 0.42
    locker_parts.append(bevel(add_box(f"locker_{i}", (0.45, 0.40, 1.85), (-4.05, y, 0.925), (0, 0, 0), M['metal']), 0.006))
    locker_parts.append(add_box(f"locker_door_{i}", (0.02, 0.36, 1.75), (-3.81, y, 0.95), (0, 0, 0), M['fabric_teal']))
    locker_parts.append(add_box(f"locker_handle_{i}", (0.02, 0.03, 0.14), (-3.79, y + 0.13, 1.15), (0, 0, 0), M['alu']))
group("lockers", locker_parts, (-4.05, 6.2, 0))
landmark("lockers", "staff lockers", "furniture", (-4.05, 6.2, 0), (0.45, 1.7, 1.85), zone="break_area")
group("noticeboard", [
    add_box("noticeboard_frame", (0.05, 1.5, 1.0), (-4.18, 8.3, 1.6), (0, 0, 0), M['wood_light']),
    add_box("noticeboard_cork", (0.02, 1.42, 0.92), (-4.14, 8.3, 1.6), (0, 0, 0), M['pot'])] +
    [add_box(f"notice_paper{i}", (0.005, 0.21, 0.297),
             (-4.12, 8.3 + random.uniform(-0.5, 0.5), 1.6 + random.uniform(-0.28, 0.28)),
             (random.uniform(-0.1, 0.1), 0, 0), M['paper']) for i in range(5)],
    (-4.14, 8.3, 1.6))
landmark("noticeboard", "notice board", "decor", (-4.14, 8.3, 1.6), (0.05, 1.5, 1.0), zone="break_area")


# ---- colour accents: acoustic felt panels, accent seating, binders, desk plants ----
FELT = [noise_bump_mat(f"felt_{n}", c, 0.95, 240, 0.6, 0.0018) for n, c in [
    ("mustard", (0.32, 0.20, 0.030)), ("rust", (0.26, 0.085, 0.035)),
    ("teal", (0.035, 0.135, 0.145)), ("sage", (0.105, 0.145, 0.085)),
    ("slate", (0.085, 0.095, 0.115)), ("sand", (0.30, 0.255, 0.175))]]


def felt_wall(name, x, y, rot, cols, rows, z0, size=0.58, gap=0.04):
    parts = []
    for r in range(rows):
        for c in range(cols):
            off = (c - (cols - 1) / 2) * (size + gap)
            parts.append(add_box(f"{name}_{r}_{c}", (size, 0.045, size),
                                 L(x, y, rot, off, 0, z0 + r * (size + gap)),
                                 (0, 0, rot), random.choice(FELT), coll="Shell"))
    return parts

felt_wall("acoustic_west", -9.96, -1.0, math.radians(90), 7, 2, 1.35)
felt_wall("acoustic_conf", 9.94, 4.0, math.radians(-90), 2, 2, 1.32)  # pier between the east windows

# accent armchairs in the lounge, both turned towards the coffee table
for i, (ax, ay, arot, fab) in enumerate([(5.5, -3.5, math.radians(160), FELT[0]),
                                         (5.5, 0.35, math.radians(20), FELT[1])]):
    ch = make_sofa(f"armchair{i}", ax, ay, arot, w=0.95, d=0.82)
    for p in ch:
        if p.data.materials and p.data.materials[0] == M['fabric_grey']:
            p.data.materials[0] = fab
    group(f"lounge_armchair_{i}", ch, (ax, ay, 0))
landmark("lounge_armchair_0", "mustard accent armchair", "furniture", (5.5, -3.5, 0), (0.95, 0.82, 0.85), zone="lounge")
landmark("lounge_armchair_1", "rust-coloured accent armchair", "furniture", (5.5, 0.35, 0), (0.95, 0.82, 0.85), zone="lounge")

# floor lamp warms up the lounge corner
lamp_parts = [add_cyl("floorlamp_base", 0.16, 0.03, (8.8, -3.4, 0.015), mat=M['metal_dark'], verts=24),
              add_cyl("floorlamp_stem", 0.018, 1.55, (8.8, -3.4, 0.79), mat=M['metal_dark'], verts=12),
              add_cone("floorlamp_shade", 0.20, 0.26, (8.8, -3.4, 1.62), rot=(math.pi, 0, 0), mat=M['plastic_white'])]
_fl = bpy.data.lights.new("floorlamp_bulb", type='POINT')
_fl.energy, _fl.color, _fl.shadow_soft_size = 22.0, (1.0, 0.80, 0.58), 0.08
_flo = bpy.data.objects.new("floorlamp_bulb", _fl)
_flo.location = (8.8, -3.4, 1.55)
link(_flo, "Lighting")
group("floor_lamp", lamp_parts, (8.8, -3.4, 0))
landmark("floor_lamp", "floor lamp", "appliance", (8.8, -3.4, 0), (0.4, 0.4, 1.75), zone="lounge")

# binders on the credenza, books on tables, small plants on desks
for i in range(7):
    add_box(f"binder_{i}", (0.055, 0.26, 0.31), (3.52, 8.05 + i * 0.065, 0.905),
            (0, random.uniform(-0.06, 0.06), math.radians(90)), random.choice(FELT))
for i in range(3):
    add_box(f"book_lounge_{i}", (0.21, 0.28, 0.035), (6.35, -1.95, ct_h + 0.018 + i * 0.036),
            (0, 0, random.uniform(-0.25, 0.25)), random.choice(FELT))
for name, dx, dy, drot, dh in [podB[1], podC[2], podA[3]]:
    px, py, _ = L(dx, dy, drot, -0.58, 0.2, 0)
    add_cyl(f"{name}_potlet", 0.065, 0.13, (px, py, dh + 0.065), mat=M['pot'], verts=16)
    add_sphere(f"{name}_potlet_leaf", 0.11, (px, py, dh + 0.17), scale=(1.2, 1.2, 0.9), mat=M['leaf'])


# ---------------------------------------------------------------------------
# Poly Haven assets
# ---------------------------------------------------------------------------

def append_collection(blend_subpath, coll_name):
    blend_path = os.path.join(ASSETS, blend_subpath)
    asset_dir = os.path.dirname(blend_path)
    before = set(bpy.data.images.keys())
    bpy.ops.wm.append(directory=os.path.join(blend_path, "Collection") + os.sep,
                      filename=coll_name, link=False)
    for iname in set(bpy.data.images.keys()) - before:
        img = bpy.data.images[iname]
        if img.filepath.startswith("//textures"):
            p = os.path.join(asset_dir, "textures", os.path.basename(img.filepath))
            if os.path.exists(p):
                img.filepath = bpy.path.relpath(p, start=BASE)
                img.reload()
    coll = bpy.data.collections.get(coll_name)
    if coll is None:
        coll = next(c for c in bpy.data.collections if c.name.startswith(coll_name))
    objs = list(coll.objects)
    for ob in objs:
        for c in list(ob.users_collection):
            c.objects.unlink(ob)
        COLLS["Props"].objects.link(ob)
    if coll.users == 0 or True:
        try:
            scene.collection.children.unlink(coll)
        except Exception:
            pass
        bpy.data.collections.remove(coll)
    return objs


def place(objs, dx, dy, dz=0.0, rot=0.0, pivot=(0.0, 0.0)):
    c, s = math.cos(rot), math.sin(rot)
    px, py = pivot
    for ob in objs:
        x, y = ob.location.x - px, ob.location.y - py
        ob.location.x = px + x * c - y * s + dx
        ob.location.y = py + x * s + y * c + dy
        ob.location.z += dz
        ob.rotation_euler.z += rot
    return objs


def copy_objs(objs, suffix):
    out = []
    for ob in objs:
        n = ob.copy()
        n.data = ob.data
        n.name = f"{ob.name}_{suffix}"
        COLLS["Props"].objects.link(n)
        out.append(n)
    return out


# rubber duck -- primary navigation target, on a workstation in pod A
duck = append_collection("rubber_duck_toy_4k.blend/rubber_duck_toy_4k.blend", "rubber_duck_toy")
_, d_wx, d_wy, d_rot, d_h = podA[0]
duck_x, duck_y, duck_z = L(d_wx, d_wy, d_rot, 0.52, -0.02, d_h)
place(duck, duck_x, duck_y, duck_z, rot=d_rot + math.radians(20))
duck[0].name = "rubber_duck_toy"
landmark("rubber_duck_toy", "rubber duck", "toy", (duck_x, duck_y, duck_z), (0.21, 0.30, 0.29),
         zone="open_plan", target=True)

# wooden display shelves -- private office + break area
shelf = append_collection("wooden_display_shelves_01_4k.blend/wooden_display_shelves_01_4k.blend",
                          "wooden_display_shelves_01")
place(shelf, -9.4, 6.6, rot=math.radians(-90))
landmark("wooden_display_shelves_01", "wooden display shelf", "furniture", (-9.4, 6.6, 0),
         (0.37, 1.08, 1.56), zone="private_office")
shelf2 = copy_objs(shelf, "b")
place(shelf2, 4.0 - (-9.4), -9.35 - 6.6, rot=math.radians(90), pivot=(-9.4, 6.6))
landmark("wooden_display_shelves_01_b", "wooden display shelf by the south wall", "furniture",
         (4.0, -9.35, 0), (1.08, 0.37, 1.56), zone="open_plan")

# baseball -- on the shelf in the private office
ball = append_collection("baseball_01_4k.blend/baseball_01_4k.blend", "baseball_01")
ball = [o for o in ball if o.type == 'MESH']
place(ball, -9.4, 6.05, 1.03)
landmark("baseball_01", "baseball", "toy", (-9.4, 6.05, 1.03), (0.075, 0.075, 0.075), zone="private_office")

# chess set -- lounge coffee table
chess = append_collection("chess_set_4k.blend/chess_set_4k.blend", "chess_set")
place(chess, 6.0, -1.6, ct_h, rot=math.radians(12))
landmark("chess_set", "chess set on the coffee table", "game", (6.0, -1.6, ct_h), (0.55, 0.55, 0.1), zone="lounge")

# school chairs -- break area round table
chair_src = append_collection("SchoolChair_01_4k.blend/SchoolChair_01_4k.blend", "SchoolChair_01")
place(chair_src, 0.6 + 0.95, 6.6, rot=math.radians(180))
landmark("SchoolChair_01", "school chair", "furniture", (1.55, 6.6, 0), (0.57, 0.68, 1.0), zone="break_area")
for i, (ang) in enumerate((90, 180, 270)):
    c = copy_objs(chair_src, f"r{i}")
    place(c, 0, 0, rot=math.radians(ang), pivot=(0.6, 6.6))


# ---------------------------------------------------------------------------
# Navigation markers: coloured / textured cubes + cones on the floor
# ---------------------------------------------------------------------------

CUBES = [
    ("cube_red", (2.2, -6.4), 0.4, M['nav_red'], "red cube"),
    ("cube_checker_green", (-3.6, -7.6), 0.4, M['nav_checker'], "green checkered cube"),
    ("cube_blue_glossy", (6.6, -7.2), 0.4, M['nav_blue'], "glossy blue cube"),
    ("cube_wood_yellow", (-8.3, -8.6), 0.4, M['wood_light'], "wooden cube"),
    ("cube_marble_orange", (1.4, 3.6), 0.4, M['nav_marble'], "orange marbled cube"),
    ("cube_stripe_purple", (8.6, 4.6), 0.4, M['nav_stripe'], "purple striped cube"),
]
for name, (x, y), s, mat, label in CUBES:
    ob = add_box(name, (s, s, s), (x, y, s / 2), (0, 0, random.uniform(0, 1.2)), mat, coll="NavMarkers")
    bevel(ob, 0.008)
    landmark(name, label, "cube", (x, y, s / 2), (s, s, s), zone=zone_at(x, y))

CONES = [
    ("cone_orange", (3.6, -0.6), M['nav_orange'], "orange traffic cone"),
    ("cone_red", (3.6, -1.7), M['nav_red_cone'], "red traffic cone"),
    ("cone_yellow", (-3.4, 2.2), M['nav_yellow'], "yellow traffic cone"),
    ("cone_white", (0.8, -5.6), M['nav_white'], "white traffic cone"),
]
for name, (x, y), mat, label in CONES:
    add_box(f"{name}_base", (0.44, 0.44, 0.035), (x, y, 0.018), mat=mat, coll="NavMarkers")
    add_cone(name, 0.19, 0.52, (x, y, 0.28), mat=mat, coll="NavMarkers")
    landmark(name, label, "cone", (x, y, 0.26), (0.44, 0.44, 0.55), zone=zone_at(x, y))


# ---------------------------------------------------------------------------
# Cameras
# ---------------------------------------------------------------------------

def add_camera(name, loc, look_at, lens=28.0, ortho_scale=None):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    if ortho_scale:
        cd.type = 'ORTHO'
        cd.ortho_scale = ortho_scale
    co = bpy.data.objects.new(name, cd)
    co.location = loc
    if look_at is not None:
        d = Vector(look_at) - Vector(loc)
        co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    link(co, "Cameras")
    return co

cam_over = add_camera("Cam_Overview", (8.9, -9.2, 2.95), (-4.5, 2.6, 0.9), lens=17)
add_camera("Cam_OpenPlan", (1.9, -5.4, 1.62), (-7.0, 0.9, 1.0), lens=24)
add_camera("Cam_Conference", (3.55, 3.55, 1.72), (7.4, 8.2, 1.0), lens=20)
add_camera("Cam_Break", (2.4, 3.9, 1.68), (-2.4, 9.2, 1.1), lens=20)
add_camera("Cam_Reception", (-3.2, -9.6, 1.72), (1.0, -7.0, 0.9), lens=24)
add_camera("Cam_Lounge", (3.6, -4.4, 1.55), (7.6, -1.2, 0.8), lens=26)
add_camera("Cam_Duck", L(d_wx, d_wy, d_rot, 0.30, -0.82, d_h + 0.40),
           (duck_x, duck_y, duck_z + 0.11), lens=50)
add_camera("Cam_TopDown", (0, 0, 16), (0, 0, 0), ortho_scale=21)
robot = add_camera("RobotCam", (-0.5, -6.5, 0.55), (-0.5, 2.0, 0.5), lens=20)
scene.camera = cam_over


# ---------------------------------------------------------------------------
# Render settings
# ---------------------------------------------------------------------------

engines = [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items]
scene.render.engine = 'BLENDER_EEVEE_NEXT' if 'BLENDER_EEVEE_NEXT' in engines else 'BLENDER_EEVEE'
scene.render.resolution_x = 1280
scene.render.resolution_y = 800
scene.render.image_settings.file_format = 'PNG'
ee = scene.eevee
for attr, val in [("taa_render_samples", 64), ("use_raytracing", True), ("use_shadows", True),
                  ("use_volumetric_lights", False), ("shadow_ray_count", 2), ("shadow_step_count", 6),
                  ("shadow_pool_size", '512'), ("light_threshold", 0.02)]:
    if hasattr(ee, attr):
        try:
            setattr(ee, attr, val)
        except Exception:
            pass
if hasattr(ee, "ray_tracing_options"):
    try:
        ee.ray_tracing_options.resolution_scale = '1'
    except Exception:
        pass
scene.view_settings.exposure = -0.55
try:
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.look = 'AgX - Medium Contrast'
except Exception:
    pass


# ---------------------------------------------------------------------------
# Save + semantic manifest
# ---------------------------------------------------------------------------

with open(os.path.join(BASE, "scene_objects.json"), "w") as f:
    json.dump({
        "description": "Semantic map of office.blend. World coordinates in metres, floor at z=0, "
                       "ceiling at 3.2 m, room interior spans x,y in [-10, 10].",
        "target_query_example": "'go to the rubber duck' -> object 'rubber_duck_toy'",
        "rooms": ROOMS,
        "objects": LANDMARKS,
    }, f, indent=2)

bpy.ops.wm.save_as_mainfile(filepath=os.path.join(BASE, "office.blend"))
print(f"BUILD OK: {len(bpy.data.objects)} objects, {len(LANDMARKS)} landmarks")
