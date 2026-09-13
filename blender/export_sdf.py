"""Export office.blend to a Gazebo Harmonic (gz-sim 8) SDF model + world.

Run:  blender -b office.blend --python export_sdf.py

Writes straight into the go2_office_sim ROS package (a sibling of this file's
blender/ directory under the go2_nav workspace root), not a local gazebo/
output here -- that package is what actually gets launched, and writing
directly into it means there's no separate copy step to remember or forget:
  ../src/go2_office_sim/models/office_scene/{model.config, model.sdf, meshes/*.glb}
  ../src/go2_office_sim/worlds/office.sdf

Design notes:
  * Visuals are glTF binary (.glb), Z-up, modifiers applied, textures downscaled.
  * Collisions are primitive <box> elements derived from each object's oriented
    bounding box -- the scene is procedurally generated from boxes/cylinders, so
    these are near-exact and far cheaper than trimesh collision in DART.
  * One link per semantic landmark (from scene_objects.json); building shell,
    lights and exterior collapse into a "structure" link; everything currently
    unlabelled (chairs, stools, chess pieces...) goes into "furniture_misc".
"""

import bpy, bmesh, os, json, math
from mathutils import Matrix, Vector

BASE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(BASE, "..", "src", "go2_office_sim")
MODEL_NAME = "office_scene"
MODEL_DIR = os.path.join(OUT, "models", MODEL_NAME)
MESH_DIR = os.path.join(MODEL_DIR, "meshes")
WORLD_DIR = os.path.join(OUT, "worlds")
SDF_VER = "1.10"

TEX_MAX = 1024               # downscale 4K Poly Haven maps before embedding
COLLISION_MIN_DIM = 0.12     # skip collision for clutter smaller than this
MIN_THICKNESS = 0.02         # planes have zero thickness; give them a slab

STRUCTURE_COLLS = {"Shell", "Lighting", "Exterior"}
NO_COLLISION_COLLS = {"Lighting", "Exterior", "Cameras"}
NO_COLLISION_NAMES = {"exterior_ground", "rug_lounge", "floor_vinyl_break"}

for d in (MESH_DIR, WORLD_DIR):
    os.makedirs(d, exist_ok=True)

with open(os.path.join(BASE, "scene_objects.json")) as f:
    manifest_doc = json.load(f)
MANIFEST = {o["name"]: o for o in manifest_doc["objects"]}


# ---------------------------------------------------------------------------
# Grouping: object -> link name
# ---------------------------------------------------------------------------

def ancestors_of(ob):
    chain = []
    cur = ob
    while cur is not None:
        chain.append(cur)
        cur = cur.parent
    return chain


def link_of(ob):
    root = ancestors_of(ob)[-1]
    if root.name in MANIFEST:
        return root.name
    base = ob.name.split(".")[0]
    if base in MANIFEST:
        return base
    coll = ob.users_collection[0].name if ob.users_collection else ""
    if coll in STRUCTURE_COLLS:
        return "structure"
    return "furniture_misc"


meshes = [o for o in bpy.data.objects if o.type == 'MESH']
groups = {}
for ob in meshes:
    groups.setdefault(link_of(ob), []).append(ob)

print(f"[export] {len(meshes)} meshes -> {len(groups)} links")


# ---------------------------------------------------------------------------
# Collision boxes (world space, computed before any re-anchoring)
# ---------------------------------------------------------------------------

def obb(ob):
    """Oriented bounding box of an object: (world centre, size, euler xyz)."""
    local_centre = sum((Vector(c) for c in ob.bound_box), Vector()) / 8.0
    centre = ob.matrix_world @ local_centre
    size = Vector(ob.dimensions)
    for i in range(3):
        if size[i] < MIN_THICKNESS:
            size[i] = MIN_THICKNESS
    return centre, size, ob.matrix_world.to_euler('XYZ')


def wants_collision(ob):
    coll = ob.users_collection[0].name if ob.users_collection else ""
    if coll in NO_COLLISION_COLLS:
        return False
    if ob.name.split(".")[0] in NO_COLLISION_NAMES:
        return False
    return max(ob.dimensions) >= COLLISION_MIN_DIM


COLLISIONS = {}   # link -> [(centre, size, euler), ...]
for link, objs in groups.items():
    COLLISIONS[link] = [obb(o) for o in objs if wants_collision(o)]
n_col = sum(len(v) for v in COLLISIONS.values())
print(f"[export] {n_col} box collisions")


# ---------------------------------------------------------------------------
# Link anchors: manifest pose where known, else footprint centre on the floor
# ---------------------------------------------------------------------------

def anchor_of(link, objs):
    if link in MANIFEST:
        return Vector(MANIFEST[link]["location"])
    if link in ("structure", "furniture_misc"):
        return Vector((0.0, 0.0, 0.0))
    pts = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return Vector(((lo.x + hi.x) / 2, (lo.y + hi.y) / 2, lo.z))


ANCHORS = {link: anchor_of(link, objs) for link, objs in groups.items()}


# ---------------------------------------------------------------------------
# Texture downscale (in-memory only; office.blend is never re-saved)
# ---------------------------------------------------------------------------

scaled = 0
for img in bpy.data.images:
    if img.source != 'FILE':
        continue
    w, h = img.size
    if max(w, h) <= TEX_MAX:
        continue
    f = TEX_MAX / max(w, h)
    try:
        # touching pixels forces the lazy-loaded buffer in before scaling
        if not img.has_data:
            img.pixels[0]
        img.scale(max(1, int(w * f)), max(1, int(h * f)))
        scaled += 1
    except (RuntimeError, IndexError) as e:
        print(f"[export]   could not scale {img.name}: {e}")
print(f"[export] downscaled {scaled} textures to <= {TEX_MAX}px")


# ---------------------------------------------------------------------------
# Flatten procedural materials
#
# glTF carries image textures + PBR scalars only. When Base Color is driven by a
# node graph the exporter drops it and writes white -- so carpet, ceiling, desks
# and the patterned cubes would all come out pure white. Resolve each procedural
# chain to a representative colour, then unlink it so the exporter emits that.
# Materials driven by an image texture are left untouched.
# ---------------------------------------------------------------------------

def upstream_colors(socket):
    """Representative colours feeding a socket; None if an image texture drives it."""
    cols, seen, stack = [], set(), [l.from_node for l in socket.links]
    while stack:
        n = stack.pop()
        if n in seen:
            continue
        seen.add(n)
        if n.type == 'TEX_IMAGE':
            return None                      # exporter handles this properly
        if n.type == 'VALTORGB':
            cols += [tuple(e.color[:3]) for e in n.color_ramp.elements]
            continue
        # for a MULTIPLY/MIX node the first colour input carries the base tone
        inputs = list(n.inputs)
        if n.type in ('MIX_RGB', 'MIX'):
            inputs = [i for i in n.inputs if i.type == 'RGBA'][:1] or inputs
        for inp in inputs:
            if inp.is_linked:
                stack += [l.from_node for l in inp.links]
            elif inp.type == 'RGBA':
                cols.append(tuple(inp.default_value[:3]))
    return cols


def flatten(mat):
    if not mat.use_nodes:
        return False
    bsdf = next((n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None)
    if bsdf is None:
        return False
    changed = False
    for key in ("Base Color", "Emission Color"):
        sock = bsdf.inputs.get(key)
        if sock is None or not sock.is_linked:
            continue
        cols = upstream_colors(sock)
        if not cols:                          # image-driven, or nothing to sample
            continue
        avg = [sum(c[i] for c in cols) / len(cols) for i in range(3)]
        for link in list(sock.links):
            mat.node_tree.links.remove(link)
        sock.default_value = (*avg, 1.0)
        changed = True
    return changed


flattened = sum(flatten(m) for m in bpy.data.materials)
print(f"[export] flattened {flattened} procedural materials to representative colours")


# ---------------------------------------------------------------------------
# Mesh export
# ---------------------------------------------------------------------------

gltf_props = set(bpy.ops.export_scene.gltf.get_rna_type().properties.keys())
GLTF_ARGS = {k: v for k, v in {
    "export_format": 'GLB',
    "use_selection": True,
    "export_apply": True,        # bake modifiers (bevels)
    "export_yup": False,         # keep Blender/Gazebo Z-up
    "export_materials": 'EXPORT',
    "export_image_format": 'JPEG',
    "export_jpeg_quality": 85,
    "export_cameras": False,
    "export_lights": False,
    "export_animations": False,
    "export_extras": False,
    "export_skins": False,
    "export_morph": False,
}.items() if k in gltf_props}


def export_link_mesh(link, objs):
    anchor = ANCHORS[link]
    roots = {ancestors_of(o)[-1] for o in objs}
    shift = Matrix.Translation(-anchor)
    for r in roots:
        r.matrix_world = shift @ r.matrix_world
    bpy.context.view_layer.update()

    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]

    path = os.path.join(MESH_DIR, f"{link}.glb")
    bpy.ops.export_scene.gltf(filepath=path, **GLTF_ARGS)

    unshift = Matrix.Translation(anchor)
    for r in roots:
        r.matrix_world = unshift @ r.matrix_world
    bpy.context.view_layer.update()
    return path


for i, (link, objs) in enumerate(sorted(groups.items()), 1):
    export_link_mesh(link, objs)
    print(f"[export] mesh {i}/{len(groups)}: {link} ({len(objs)} objects)")


# ---------------------------------------------------------------------------
# SDF emission
# ---------------------------------------------------------------------------

def pose(v, e=(0.0, 0.0, 0.0)):
    return (f"{v[0]:.5f} {v[1]:.5f} {v[2]:.5f} "
            f"{e[0]:.5f} {e[1]:.5f} {e[2]:.5f}")


def link_xml(link, objs):
    a = ANCHORS[link]
    lm = MANIFEST.get(link)
    out = [f'    <link name="{link}">']
    out.append(f'      <pose>{pose(a)}</pose>')
    if lm:
        out.append(f'      <!-- {lm["semantic_label"]} | {lm["category"]} | zone: {lm["zone"]}'
                   f'{" | TARGET" if lm["is_target"] else ""} -->')
    out.append(f'      <visual name="{link}_visual">')
    out.append('        <geometry><mesh>')
    out.append(f'          <uri>model://{MODEL_NAME}/meshes/{link}.glb</uri>')
    out.append('        </mesh></geometry>')
    out.append('      </visual>')
    for j, (c, s, e) in enumerate(COLLISIONS[link]):
        rel = c - a
        out.append(f'      <collision name="{link}_c{j}">')
        out.append(f'        <pose>{pose(rel, e)}</pose>')
        out.append(f'        <geometry><box><size>{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}</size></box></geometry>')
        out.append('      </collision>')
    out.append('    </link>')
    return "\n".join(out)


model_sdf = [f'<?xml version="1.0" ?>',
             f'<sdf version="{SDF_VER}">',
             f'  <model name="{MODEL_NAME}">',
             f'    <static>true</static>']
for link, objs in sorted(groups.items()):
    model_sdf.append(link_xml(link, objs))
model_sdf += ['  </model>', '</sdf>', '']
with open(os.path.join(MODEL_DIR, "model.sdf"), "w") as f:
    f.write("\n".join(model_sdf))

with open(os.path.join(MODEL_DIR, "model.config"), "w") as f:
    f.write(f"""<?xml version="1.0" ?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="{SDF_VER}">model.sdf</sdf>
  <description>
    Open-plan office generated from office.blend for semantic robot navigation.
    {len(groups)} links, {n_col} primitive collisions. Semantic metadata in
    scene_objects.json (room bounds, per-landmark labels, navigation target).
  </description>
</model>
""")


# ---- lights: SDF has no area lights, so panels become point lights ----
light_xml = []
sun = bpy.data.objects.get("sun")
if sun:
    d = (sun.matrix_world.to_quaternion() @ Vector((0.0, 0.0, -1.0))).normalized()
    light_xml.append(f"""    <light type="directional" name="sun">
      <pose>0 0 12 0 0 0</pose>
      <direction>{d.x:.4f} {d.y:.4f} {d.z:.4f}</direction>
      <diffuse>1.0 0.96 0.90 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <cast_shadows>true</cast_shadows>
    </light>""")

panels = [o for o in bpy.data.objects if o.type == 'LIGHT' and o.data.type == 'AREA']
for o in sorted(panels, key=lambda x: x.name):
    p = o.matrix_world.translation
    light_xml.append(f"""    <light type="point" name="{o.name}">
      <pose>{p.x:.3f} {p.y:.3f} {p.z:.3f} 0 0 0</pose>
      <diffuse>0.85 0.80 0.72 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>9</range><linear>0.25</linear><quadratic>0.02</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")

world_sdf = f"""<?xml version="1.0" ?>
<sdf version="{SDF_VER}">
  <world name="office">

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.45 0.45 0.48 1</ambient>
      <background>0.72 0.80 0.89 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>8.9 -9.2 2.95 0 0.25 2.2</pose>
      </camera>
    </gui>

{chr(10).join(light_xml)}

    <include>
      <uri>model://{MODEL_NAME}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

  </world>
</sdf>
"""
with open(os.path.join(WORLD_DIR, "office.sdf"), "w") as f:
    f.write(world_sdf)

total_mb = sum(os.path.getsize(os.path.join(MESH_DIR, f))
               for f in os.listdir(MESH_DIR)) / 1e6
print(f"[export] DONE: {len(groups)} links, {n_col} collisions, "
      f"{len(light_xml)} lights, meshes {total_mb:.1f} MB")
print(f"[export] world: {os.path.join(WORLD_DIR, 'office.sdf')}")
