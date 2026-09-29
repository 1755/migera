# Gothic 2 World Export

Exports a ZenGin world (e.g. `NEWWORLD.ZEN`, Khorinis) from a **Gothic II /
Night of the Raven** install to a single `.glb` for import into Bevy or
Blender. Includes geometry, PBR materials, base-color textures, UV coordinates,
and alpha transparency — all derived from the game's own data.

You need a legitimate, already-installed copy of Gothic II. This tool reads
your local `Data/*.vdf` archives; it does not download or redistribute any
game content.

## What gets exported

- The compiled static **world mesh** (terrain, buildings-as-world-geometry),
  as one node per material group.
- Every placed **vob** (prop/decoration/building object) that has a mesh or
  model visual (`.MRM` / `.MDM`), as a child node with its world
  position/rotation. Vobs with no visual (triggers, zones, sound emitters,
  etc.) are skipped, as are decal/particle/camera visuals.

### Materials

Each ZenGin material is converted to a glTF PBR metallic-roughness material:

| ZenGin property | glTF property | Notes |
|---|---|---|
| `color` (RGBA) | `baseColorFactor` | Material tint, applied as a multiply |
| `texture` | `baseColorTexture` | Loaded from VDF archives, embedded as PNG |
| `environment_mapping` + strength | `metallicFactor` | Gothic's reflectivity hack |
| `MaterialGroup` | `roughnessFactor` | Per-group lookup (metal=0.35, water=0.15, etc.) |
| `alpha_function` | `alphaMode` | NONE/DEFAULT→OPAQUE, BLEND/ADD→BLEND, etc. |
| (derived) | `doubleSided` | Auto-set for alpha-blended materials |

UV coordinates (`TEXCOORD_0`) are exported for both the world mesh (from
`Feature.texture`) and vob meshes (from `MeshWedge.texture`).

### Not exported

Animated skeletal poses (only bind-pose geometry of `.MDM` model meshes), NPCs,
lighting, collision, normal maps (ZenGin doesn't ship any), LOD selection
(uses whatever the base `.MRM`/world mesh stores).

## Setup (Nix)

From the repo root:

```
nix develop .#gothic-export
```

First run creates `tools/gothic_export/.venv` and installs dependencies
automatically. Then:

```
source tools/gothic_export/.venv/bin/activate
```

This shell exists separately from the main Bevy `nix develop` shell — it's
not needed to build/run the game, only to run this export script.

### Non-Nix setup

Requires **Python 3.10–3.13** (zenkit's ctypes bindings do not yet support
3.14 — `pip install zenkit` will import-error with
`cannot import name 'POINTER' from '_ctypes'` on 3.14).

```
python3.12 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

```
python export_world.py --gothic-dir "/path/to/Gothic II" --world NEWWORLD.ZEN --out khorinis.glb
```

`--gothic-dir` must contain a `Data/` subfolder with the game's `.vdf`
archives (this is standard for any Gothic II install — Steam/GOG/original
disc all lay it out this way).

Options:
- `--world NAME.ZEN` — world to export (default `NEWWORLD.ZEN`). Other
  Gothic worlds (`OLDWORLD.ZEN`, `ADDONWORLD.ZEN` for Night of the Raven,
  etc.) work the same way if present in the VDFs.
- `--world-mesh-only` — skip the vob walk, export just the static world
  shell. Much faster; useful for a first test.
- `-v` — verbose logging (shows every VDF mount, texture loads, and any mesh
  lookup misses).

## Importing into Bevy

The output is a standard `.glb`; drop it under `assets/` and load with
`asset_server.load("khorinis.glb#Scene0")` via `bevy_gltf` (already pulled
in by the `bevy` dependency in this repo — no extra crate needed).

Materials use the standard glTF PBR metallic-roughness model, so Bevy's
`StandardMaterial` picks them up automatically — including base-color textures,
alpha blending, and double-sided rendering.

Coordinate system: ZenGin is Z-up/left-handed/centimeters; the script
converts to Y-up/right-handed/meters (glTF/Bevy convention) on export, so no
further transform should be needed on import.

## Known limitations / troubleshooting

- **Missing textures in log output** (`Texture X not found in VDFs`): the
  material references a texture file that isn't in your mounted VDFs — usually
  means an addon/mod VDF is missing. The material falls back to its flat color
  factor.
- **Missing meshes in log output** (`Could not find X.MRM in VDFs`): the vob
  references a visual that isn't in your mounted VDFs — usually means an
  addon/mod VDF is missing, or the visual is a procedural/skeletal type this
  script doesn't resolve (e.g. pure `.MDS` animation-driven visuals without a
  baked `.MDM`).
- **Very large output**: Khorinis is a big map — expect the full export
  (world mesh + vobs) to be tens to low hundreds of MB and take a couple of
  minutes. Textures add to the size. Use `--world-mesh-only` to iterate faster.
- **Animated/scrolling textures**: ZenGin supports texture scrolling (water,
  lava). The exporter captures the base texture but not the animation — the
  texture will appear static in the exported glb.
