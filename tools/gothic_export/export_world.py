#!/usr/bin/env python3
"""
Export a Gothic 2 ZenGin world (e.g. NEWWORLD.ZEN / Khorinis) to glTF (.glb).

The compiled world mesh plus every placed vob's visual mesh, positioned as glTF
nodes. Materials include base-color textures (loaded from the game's VDF
archives), PBR metallic/roughness derived from ZenGin material properties, and
alpha transparency modes.

Requires the game's Data/*.vdf archives (a legitimate Gothic 2 / Night of the
Raven install). Requires Python 3.11-3.13 (zenkit's ctypes bindings do not
yet support 3.14) - see README.md for the Nix venv setup used in this repo.

Usage:
    python export_world.py --gothic-dir "/path/to/Gothic II" --world NEWWORLD.ZEN --out khorinis.glb
"""
from __future__ import annotations

import argparse
import io
import logging
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

try:
    import zenkit
except ImportError:
    sys.exit("Missing dependency 'zenkit'. Run: pip install -r requirements.txt")

try:
    from pygltflib import (
        ARRAY_BUFFER,
        BLEND,
        ELEMENT_ARRAY_BUFFER,
        FLOAT,
        MASK,
        OPAQUE,
        UNSIGNED_INT,
        Accessor,
        Asset,
        Buffer,
        BufferView,
        GLTF2,
        Image as GltfImage,
        Material as GltfMaterial,
        Mesh as GltfMesh,
        Node,
        PbrMetallicRoughness,
        Primitive,
        Sampler,
        Scene,
        Texture as GltfTexture,
        TextureInfo,
    )
except ImportError:
    sys.exit("Missing dependency 'pygltflib'. Run: pip install -r requirements.txt")

log = logging.getLogger("gothic_export")

# Silence noisy PIL/Pillow debug messages (STREAM blocks, etc.)
logging.getLogger("PIL").setLevel(logging.WARNING)

# ZenGin is Y-up, left-handed, centimeters (confirmed empirically: NEWWORLD.ZEN's
# raw Y span is ~406m vs. X/Z spans of ~1555m/~1706m - Y is the small "height" axis).
# glTF is Y-up, right-handed, meters. Same up-axis, so only a handedness flip
# (negate Z) plus unit scale is needed - no axis swap.
UNIT_SCALE = 1.0 / 100.0

# Extensions to try when looking up a texture filename in the VDF.
_TEX_EXTENSIONS = ["", ".TGA", ".TEX", ".PCX", ".tga", ".tex", ".pcx"]

# ZenGin alpha function → glTF alpha mode.
_ALPHA_MODE_MAP = {
    zenkit.AlphaFunction.NONE: OPAQUE,
    zenkit.AlphaFunction.DEFAULT: OPAQUE,
    zenkit.AlphaFunction.BLEND: BLEND,
    zenkit.AlphaFunction.ADD: BLEND,
    zenkit.AlphaFunction.SUBTRACT: BLEND,
    zenkit.AlphaFunction.MULTIPLY: BLEND,
    zenkit.AlphaFunction.MULTIPLY_ALT: BLEND,
}


# ---------------------------------------------------------------------------
# Material model
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MaterialInfo:
    """PBR properties derived from a ZenGin material, for glTF export.

    ZenGin ships no PBR data (it predates that shading model), but its material
    fields map onto it reasonably:
    - ``color`` → ``baseColorFactor`` (RGBA tint, applied as a multiply)
    - ``MaterialGroup`` → ``roughnessFactor`` (coarse but real physical class)
    - ``environment_mapping`` + strength → ``metallicFactor`` (Gothic's
      reflectivity hack)
    - ``texture`` → ``baseColorTexture`` (loaded from VDF, embedded as PNG)
    - ``alpha_function`` → ``alphaMode`` (OPAQUE / MASK / BLEND)
    """

    name: str
    color: tuple[float, float, float, float]
    metallic: float
    roughness: float
    texture_png: bytes | None = None
    alpha_mode: str = OPAQUE
    alpha_cutoff: float = 0.5
    double_sided: bool = False


# Per-MaterialGroup roughness: metal and water are the shiny surfaces in ZenGin's
# groups, everything else is diffuse.  Values are a plausible-looking default, not
# measured - ZenGin has no roughness concept of its own.
_GROUP_ROUGHNESS = {
    zenkit.MaterialGroup.METAL: 0.35,
    zenkit.MaterialGroup.WATER: 0.15,
    zenkit.MaterialGroup.STONE: 0.9,
    zenkit.MaterialGroup.WOOD: 0.75,
    zenkit.MaterialGroup.EARTH: 0.95,
    zenkit.MaterialGroup.SNOW: 0.85,
    zenkit.MaterialGroup.UNDEFINED: 0.8,
}


def _png_has_significant_transparency(png_data: bytes) -> bool:
    """Return True if the texture has enough transparent pixels to warrant BLEND mode.

    A single stray transparent pixel (e.g. from texture-atlas padding) should not
    make an entire barrel or roof semi-transparent.  We require at least 2% of
    pixels to have alpha < 128 before upgrading OPAQUE → BLEND.
    """
    img = PILImage.open(io.BytesIO(png_data))
    if img.mode != "RGBA":
        return False
    alpha = np.array(img)[:, :, 3]
    total = alpha.size
    transparent = int(np.sum(alpha < 128))
    return transparent / total > 0.02


def _srgb_to_linear(c: float) -> float:
    """Convert a single sRGB component (0-1) to linear (0-1)."""
    if c <= 0.04045:
        return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4


def material_info(
    mat: zenkit.Material | None,
    fallback_name: str,
    tex_cache: TextureCache | None = None,
) -> MaterialInfo:
    """Derive a ``MaterialInfo`` from a ZenGin material, optionally loading its texture."""
    if mat is None:
        return MaterialInfo(
            name=fallback_name,
            color=(0.7, 0.7, 0.7, 1.0),
            metallic=0.0,
            roughness=0.8,
        )

    # ZenGin material colors are sRGB (0-255).  glTF baseColorFactor must be
    # LINEAR — passing sRGB values as-is makes everything 3-5x too bright,
    # causing the "white shading" wash-out on barrels, trees, roofs, etc.
    color = (
        _srgb_to_linear(mat.color.r / 255.0),
        _srgb_to_linear(mat.color.g / 255.0),
        _srgb_to_linear(mat.color.b / 255.0),
        mat.color.a / 255.0,  # alpha stays as-is (linear in GLTF)
    )
    metallic = mat.environment_mapping_strength if mat.environment_mapping else 0.0
    roughness = _GROUP_ROUGHNESS.get(mat.group, 0.8)

    # Alpha mode from ZenGin's alpha function.
    alpha_mode = _ALPHA_MODE_MAP.get(mat.alpha_function, OPAQUE)

    # Double-sided for alpha-blended materials (foliage, glass, etc.) and for
    # materials that ZenGin marks as two-sided via the alpha function.
    double_sided = alpha_mode != OPAQUE

    # Load the base-color texture from the VDF archives.
    texture_png: bytes | None = None
    if tex_cache is not None and mat.texture:
        texture_png = tex_cache.load(mat.texture)

    # Gothic 2 stores many alpha-tested materials (foliage, fences, flags,
    # etc.) with alpha_function=NONE — the engine reads transparency from the
    # texture's alpha channel directly.  If the material was classified as
    # OPAQUE but the texture actually has significant transparent pixels,
    # upgrade to MASK mode so the renderer discards transparent fragments
    # while still writing depth (fixing z-order sorting vs BLEND).
    alpha_cutoff = 0.5
    if alpha_mode == OPAQUE and texture_png is not None:
        if _png_has_significant_transparency(texture_png):
            log.debug("Material %s: OPAQUE → MASK (texture has alpha)", fallback_name)
            alpha_mode = MASK
            double_sided = True

    return MaterialInfo(
        name=mat.name,
        color=color,
        metallic=metallic,
        roughness=roughness,
        texture_png=texture_png,
        alpha_mode=alpha_mode,
        alpha_cutoff=alpha_cutoff,
        double_sided=double_sided,
    )


# ---------------------------------------------------------------------------
# Texture loading
# ---------------------------------------------------------------------------

class TextureCache:
    """Loads and caches textures from Gothic 2's VDF archives as PNG bytes.

    ZenGin textures (.TGA, .TEX, .PCX) are decoded to RGBA via zenkit and then
    re-encoded to PNG for embedding in the glTF binary.
    """

    def __init__(self, vfs: zenkit.Vfs) -> None:
        self._vfs = vfs
        self._cache: dict[str, bytes | None] = {}

    def load(self, name: str) -> bytes | None:
        """Load a texture by filename and return PNG bytes, or None on failure."""
        key = name.upper()
        if key in self._cache:
            return self._cache[key]

        node = self._find_texture(name)

        if node is None:
            log.debug("Texture %s not found in VDFs", name)
            self._cache[key] = None
            return None

        try:
            tex = zenkit.Texture.load(node)
            rgba = tex.mipmap_rgba(0)
            img = PILImage.frombytes("RGBA", (tex.width, tex.height), rgba)
            buf = io.BytesIO()
            img.save(buf, format="PNG", optimize=True)
            png_data = buf.getvalue()
            log.debug("Loaded texture %s (%dx%d, %d bytes PNG)", name, tex.width, tex.height, len(png_data))
            self._cache[key] = png_data
            return png_data
        except Exception as exc:
            log.warning("Failed to load texture %s: %s", name, exc)
            self._cache[key] = None
            return None

    def _find_texture(self, name: str) -> zenkit.VfsNode | None:
        """Resolve a material texture name to a VfsNode in the VDF archives.

        Gothic 2 material textures are referenced as ``FOO.TGA`` (or sometimes
        without extension) but the compiled VDF archives store them as
        ``FOO-C.TEX``.  We try several lookup strategies in order.
        """
        stem = Path(name).stem.upper()

        # 1. Exact match (rare, but free)
        node = self._vfs.find(name.upper())
        if node is not None:
            return node

        # 2. Compiled color texture: stem-C.TEX (covers ~99% of cases)
        node = self._vfs.find(stem + "-C.TEX")
        if node is not None:
            return node

        # 3. Stem without extension + various extensions
        for ext in _TEX_EXTENSIONS:
            node = self._vfs.find(stem + ext)
            if node is not None:
                return node

        return None


# ---------------------------------------------------------------------------
# Geometry data structures
# ---------------------------------------------------------------------------

@dataclass
class MeshPart:
    """One draw call worth of geometry: positions/normals/uvs/indices, single material."""

    material: MaterialInfo
    positions: np.ndarray  # (N, 3) float32
    normals: np.ndarray  # (N, 3) float32
    indices: np.ndarray  # (M,) uint32
    uvs: np.ndarray | None = None  # (N, 2) float32, optional


@dataclass
class SceneNode:
    name: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]  # xyzw
    parts: list[MeshPart] = field(default_factory=list)
    children: list["SceneNode"] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

def zengin_pos_to_gltf(v: zenkit.Vec3f) -> tuple[float, float, float]:
    # Left-handed -> right-handed: negate Z, keep Y up.
    return (v.x * UNIT_SCALE, v.y * UNIT_SCALE, -v.z * UNIT_SCALE)


def zengin_dir_to_gltf(v: zenkit.Vec3f) -> tuple[float, float, float]:
    return (v.x, v.y, -v.z)


def zengin_rot_to_gltf_quat(rot: zenkit.Mat3x3) -> tuple[float, float, float, float]:
    """Convert a ZenGin (left-handed) rotation matrix to a glTF (right-handed) quaternion.

    Mirroring a single axis (Z, to match `zengin_pos_to_gltf`) turns a proper LH
    rotation matrix into an improper one (det = -1) if applied naively to its
    columns, which has no quaternion representation. The correct conversion is a
    similarity transform R_rh = M @ R_lh @ M with M = diag(1, 1, -1), which
    simplifies to negating every matrix entry touched by row 2 or column 2 exactly
    once (m02, m20, m12, m21) - verified against zenkit's raw matrix columns for
    known vob rotations before landing on this formula (see PR discussion / git
    history for the derivation; a naive component-negation on the *quaternion*
    output of `to_quaternion()` does not work in general).
    """
    cols = rot.columns
    m = [[getattr(cols[c], axis) for c in range(3)] for axis in ("x", "y", "z")]
    m[0][2] *= -1
    m[2][0] *= -1
    m[1][2] *= -1
    m[2][1] *= -1

    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (x, y, z, w)


# ---------------------------------------------------------------------------
# Geometry extraction — world mesh (BSP compiled terrain)
# ---------------------------------------------------------------------------

def _fan_triangulate(n: int) -> list[tuple[int, int, int]]:
    """Fan-triangulate a convex n-gon starting from vertex 0."""
    return [(0, i, i + 1) for i in range(1, n - 1)]


def _flat_normals(positions: np.ndarray, indices: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(positions)
    tris = indices.reshape(-1, 3)
    for a, b, c in tris:
        edge1 = positions[b] - positions[a]
        edge2 = positions[c] - positions[a]
        n = np.cross(edge1, edge2)
        norm = np.linalg.norm(n)
        if norm > 1e-8:
            n = n / norm
        normals[a] += n
        normals[b] += n
        normals[c] += n
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths < 1e-8] = 1.0
    return (normals / lengths).astype(np.float32)


def build_mesh_parts_from_world_mesh(
    mesh: zenkit.Mesh,
    tex_cache: TextureCache | None = None,
) -> list[MeshPart]:
    """The compiled BSP world mesh: n-gon polygon soup, each tagged with a material index.

    Unlike the previous version which deduplicated vertices per-material (losing
    UV uniqueness), this builds one MeshPart per material with per-polygon-vertex
    storage so that UV coordinates are never shared across polygon edges.
    """
    all_positions = mesh.positions
    all_features = mesh.features
    materials = mesh.materials

    # Identify portal / invisible polygon materials to skip.
    # Portal polygons have names starting with "P:" (BSP sector boundary planes)
    # and "_NIXDRAUF" ("nothing on it") — these are invisible partition geometry
    # that was never rendered in the original engine but shows up as solid
    # covering meshes when exported naively.
    skip_mat_indices: set[int] = set()
    for i, mat in enumerate(materials):
        if mat.name.startswith("P:") or mat.name == "_NIXDRAUF":
            skip_mat_indices.add(i)

    # Group polygons by material index.
    by_material: dict[int, list[zenkit.Polygon]] = {}
    for poly in mesh.polygons:
        if poly.material_index in skip_mat_indices:
            continue
        by_material.setdefault(poly.material_index, []).append(poly)

    parts: list[MeshPart] = []
    for mat_idx, polys in by_material.items():
        positions_list: list[tuple[float, float, float]] = []
        normals_list: list[tuple[float, float, float]] = []
        uvs_list: list[tuple[float, float]] = []
        indices: list[int] = []

        for poly in polys:
            pos_idx = poly.position_indices
            feat_idx = poly.feature_indices
            n = len(pos_idx)
            if n < 3:
                continue

            base = len(positions_list)
            for i in range(n):
                positions_list.append(zengin_pos_to_gltf(all_positions[pos_idx[i]]))
                feat = all_features[feat_idx[i]]
                normals_list.append(zengin_dir_to_gltf(feat.normal))
                uvs_list.append((feat.texture.x, feat.texture.y))

            for a, b, c in _fan_triangulate(n):
                indices.extend([base + a, base + b, base + c])

        if not indices:
            continue

        pos = np.array(positions_list, dtype=np.float32)
        nrm = np.array(normals_list, dtype=np.float32)
        uvs = np.array(uvs_list, dtype=np.float32)
        idx = np.array(indices, dtype=np.uint32)

        mat_obj = materials[mat_idx] if 0 <= mat_idx < len(materials) else None
        info = material_info(mat_obj, fallback_name=f"material_{mat_idx}", tex_cache=tex_cache)
        parts.append(MeshPart(material=info, positions=pos, normals=nrm, indices=idx, uvs=uvs))

    return parts


# ---------------------------------------------------------------------------
# Geometry extraction — vob meshes (MRM / MDM props)
# ---------------------------------------------------------------------------

def build_mesh_parts_from_mrm(
    mrm: zenkit.MultiResolutionMesh,
    tex_cache: TextureCache | None = None,
) -> list[MeshPart]:
    """A prop/vob mesh (.MRM): one SubMesh per material.  Wedge.index points into mrm.positions."""
    base_positions = mrm.positions
    parts: list[MeshPart] = []
    for sub in mrm.submeshes:
        wedges = sub.wedges
        if not wedges:
            continue
        pos = np.array([zengin_pos_to_gltf(base_positions[w.index]) for w in wedges], dtype=np.float32)
        nrm = np.array([zengin_dir_to_gltf(w.normal) for w in wedges], dtype=np.float32)
        uvs = np.array([(w.texture.x, w.texture.y) for w in wedges], dtype=np.float32)
        idx_list = []
        for tri in sub.triangles:
            idx_list.extend(tri.wedges)
        if not idx_list:
            continue
        idx = np.array(idx_list, dtype=np.uint32)
        info = material_info(sub.material, fallback_name="material", tex_cache=tex_cache)
        parts.append(MeshPart(material=info, positions=pos, normals=nrm, indices=idx, uvs=uvs))
    return parts


def build_mesh_parts_from_model_mesh(
    model_mesh: zenkit.ModelMesh,
    tex_cache: TextureCache | None = None,
) -> list[MeshPart]:
    parts: list[MeshPart] = []
    for soft_skin in model_mesh.meshes:
        parts.extend(build_mesh_parts_from_mrm(soft_skin.mesh, tex_cache))
    for name, mrm in model_mesh.attachments.items():
        parts.extend(build_mesh_parts_from_mrm(mrm, tex_cache))
    return parts


def resolve_visual_mesh_parts(
    vfs: zenkit.Vfs,
    visual: zenkit.Visual,
    cache: dict,
    tex_cache: TextureCache | None = None,
) -> list[MeshPart]:
    if visual is None:
        return []

    stem = Path(visual.name).stem.upper()
    if stem in cache:
        return cache[stem]

    parts: list[MeshPart] = []
    vtype = visual.type
    if vtype in (zenkit.VisualType.MULTI_RESOLUTION_MESH, zenkit.VisualType.MESH):
        node = vfs.find(f"{stem}.MRM")
        if node is None:
            log.warning("Could not find %s.MRM in VDFs (visual: %s)", stem, visual.name)
        else:
            try:
                parts = build_mesh_parts_from_mrm(zenkit.MultiResolutionMesh.load(node.open()), tex_cache)
            except Exception as exc:
                log.warning("Failed to load %s.MRM: %s", stem, exc)
    elif vtype == zenkit.VisualType.MODEL:
        node = vfs.find(f"{stem}.MDM")
        if node is not None:
            try:
                parts = build_mesh_parts_from_model_mesh(zenkit.ModelMesh.load(node.open()), tex_cache)
            except Exception as exc:
                log.warning("Failed to load %s.MDM: %s", stem, exc)
        else:
            # Some models ship as a combined .MDL (mesh + hierarchy) instead of a
            # standalone .MDM alongside a .MDH hierarchy.
            node = vfs.find(f"{stem}.MDL")
            if node is None:
                log.warning("Could not find %s.MDM or .MDL in VDFs (visual: %s)", stem, visual.name)
            else:
                try:
                    parts = build_mesh_parts_from_model_mesh(zenkit.Model.load(node.open()).mesh, tex_cache)
                except Exception as exc:
                    log.warning("Failed to load %s.MDL: %s", stem, exc)
    # DECAL / PARTICLE_EFFECT / MORPH_MESH / CAMERA visuals are not geometry we export.

    cache[stem] = parts
    return parts


def walk_vobs(
    vfs: zenkit.Vfs,
    vobs,
    cache: dict,
    stats: dict,
    tex_cache: TextureCache | None = None,
) -> list[SceneNode]:
    # ZenGin archives each vob's FULL object-to-world trafo (`trafoObjToWorld`);
    # the vob tree is logical grouping only.  zenkit's Vob.position/rotation are
    # therefore already world-space (verified: nested vob positions fall inside
    # their own world-space bbox).  Nesting them hierarchically in glTF would
    # compose ancestor transforms on top, flinging non-root vobs far away and
    # high into the sky — so the tree is flattened here instead.
    nodes = []
    for vob in vobs:
        parts = resolve_visual_mesh_parts(vfs, vob.visual, cache, tex_cache)
        if parts:
            stats["vobs_with_mesh"] += 1
        nodes.extend(walk_vobs(vfs, vob.children, cache, stats, tex_cache))
        if not parts:
            continue
        nodes.append(
            SceneNode(
                name=vob.name or vob.preset_name or f"vob_{vob.id}",
                translation=zengin_pos_to_gltf(vob.position),
                rotation=zengin_rot_to_gltf_quat(vob.rotation),
                parts=parts,
            )
        )
    return nodes


# ---------------------------------------------------------------------------
# GLB builder — accumulates geometry + textures into a single .glb
# ---------------------------------------------------------------------------

class GlbBuilder:
    """Builds a glTF 2.0 Binary (.glb) from MeshParts and a node hierarchy."""

    def __init__(self) -> None:
        self.gltf = GLTF2(asset=Asset(generator="gothic-export.py"))
        self.gltf.scenes.append(Scene(nodes=[]))
        self.gltf.scene = 0
        self._blob = bytearray()
        # Material dedup: material name → glTF material index.
        self._material_indices: dict[str, int] = {}
        # Texture dedup: PNG data id → glTF texture index.
        self._texture_indices: dict[int, int] = {}
        # Image dedup: PNG data id → glTF image index.
        self._image_indices: dict[int, int] = {}
        # Sampler index (shared by all textures).
        self._sampler_idx: int | None = None

    def _add_buffer_view(self, data: bytes, target: int | None = None) -> int:
        """Append *data* to the binary blob and create a BufferView.

        For geometry, *target* is ``ARRAY_BUFFER`` or ``ELEMENT_ARRAY_BUFFER``.
        For embedded images, *target* is ``None`` (omitted from the glTF).
        """
        offset = len(self._blob)
        self._blob.extend(data)
        while len(self._blob) % 4 != 0:
            self._blob.append(0)
        self.gltf.bufferViews.append(
            BufferView(buffer=0, byteOffset=offset, byteLength=len(data), target=target)
        )
        return len(self.gltf.bufferViews) - 1

    def _ensure_sampler(self) -> int:
        if self._sampler_idx is None:
            self.gltf.samplers.append(Sampler())  # defaults: REPEAT wrap, LINEAR filter
            self._sampler_idx = len(self.gltf.samplers) - 1
        return self._sampler_idx

    def _ensure_texture(self, png_data: bytes) -> int:
        """Return the glTF texture index for the given PNG bytes, creating if needed."""
        key = id(png_data)
        if key in self._texture_indices:
            return self._texture_indices[key]

        # Create the Image (embedded in the GLB via a bufferView).
        image_bv = self._add_buffer_view(png_data)  # no target for images
        image_idx = len(self.gltf.images)
        self.gltf.images.append(GltfImage(bufferView=image_bv, mimeType="image/png"))
        self._image_indices[key] = image_idx

        # Create the Texture referencing the image and a shared sampler.
        tex_idx = len(self.gltf.textures)
        self.gltf.textures.append(GltfTexture(sampler=self._ensure_sampler(), source=image_idx))
        self._texture_indices[key] = tex_idx
        return tex_idx

    def _material_index(self, info: MaterialInfo) -> int:
        """Return the glTF material index for a MaterialInfo, creating if needed."""
        key = self._material_key(info)
        if key not in self._material_indices:
            pbr = PbrMetallicRoughness(
                baseColorFactor=list(info.color),
                metallicFactor=info.metallic,
                roughnessFactor=info.roughness,
            )
            if info.texture_png is not None:
                tex_idx = self._ensure_texture(info.texture_png)
                pbr.baseColorTexture = TextureInfo(index=tex_idx)

            self.gltf.materials.append(
                GltfMaterial(
                    name=info.name,
                    pbrMetallicRoughness=pbr,
                    alphaMode=info.alpha_mode,
                    alphaCutoff=info.alpha_cutoff if info.alpha_mode == MASK else None,
                    doubleSided=info.double_sided,
                )
            )
            self._material_indices[key] = len(self.gltf.materials) - 1
        return self._material_indices[key]

    @staticmethod
    def _material_key(info: MaterialInfo) -> str:
        """Unique key that accounts for name, texture, color, and PBR properties.

        Many ZenGin materials share generic names like "material" or "material_13".
        Using only the name would cause incorrect de-duplication — two materials
        with the same name but different textures would share one GLTF material.
        """
        tex_hash = hash(info.texture_png) if info.texture_png is not None else 0
        return (
            f"{info.name}|{tex_hash}|"
            f"{info.color[0]:.4f},{info.color[1]:.4f},{info.color[2]:.4f},{info.color[3]:.4f}|"
            f"{info.metallic:.4f}|{info.roughness:.4f}|"
            f"{info.alpha_mode}|{info.double_sided}"
        )

    def _add_mesh(self, parts: list[MeshPart]) -> int:
        primitives = []
        for part in parts:
            # --- Position accessor ---
            pos_bv = self._add_buffer_view(part.positions.astype("<f4").tobytes(), ARRAY_BUFFER)
            self.gltf.accessors.append(
                Accessor(
                    bufferView=pos_bv,
                    componentType=FLOAT,
                    count=len(part.positions),
                    type="VEC3",
                    min=part.positions.min(axis=0).tolist(),
                    max=part.positions.max(axis=0).tolist(),
                )
            )
            pos_acc = len(self.gltf.accessors) - 1

            # --- Normal accessor ---
            nrm_bv = self._add_buffer_view(part.normals.astype("<f4").tobytes(), ARRAY_BUFFER)
            self.gltf.accessors.append(
                Accessor(bufferView=nrm_bv, componentType=FLOAT, count=len(part.normals), type="VEC3")
            )
            nrm_acc = len(self.gltf.accessors) - 1

            # --- Index accessor ---
            idx_bv = self._add_buffer_view(part.indices.astype("<u4").tobytes(), ELEMENT_ARRAY_BUFFER)
            self.gltf.accessors.append(
                Accessor(bufferView=idx_bv, componentType=UNSIGNED_INT, count=len(part.indices), type="SCALAR")
            )
            idx_acc = len(self.gltf.accessors) - 1

            # --- Attributes ---
            attributes: dict[str, int] = {"POSITION": pos_acc, "NORMAL": nrm_acc}

            # --- UV accessor (TEXCOORD_0) ---
            if part.uvs is not None and len(part.uvs) == len(part.positions):
                uv_bv = self._add_buffer_view(part.uvs.astype("<f4").tobytes(), ARRAY_BUFFER)
                self.gltf.accessors.append(
                    Accessor(bufferView=uv_bv, componentType=FLOAT, count=len(part.uvs), type="VEC2")
                )
                attributes["TEXCOORD_0"] = len(self.gltf.accessors) - 1

            primitives.append(
                Primitive(
                    attributes=attributes,
                    indices=idx_acc,
                    material=self._material_index(part.material),
                )
            )
        self.gltf.meshes.append(GltfMesh(primitives=primitives))
        return len(self.gltf.meshes) - 1

    def add_node(self, node: SceneNode, parent_children: list[int]) -> None:
        gltf_node = Node(name=node.name, translation=list(node.translation), rotation=list(node.rotation))
        if node.parts:
            gltf_node.mesh = self._add_mesh(node.parts)
        self.gltf.nodes.append(gltf_node)
        idx = len(self.gltf.nodes) - 1
        parent_children.append(idx)

        gltf_node.children = []
        for child in node.children:
            self.add_node(child, gltf_node.children)

    def finalize(self, out_path: Path) -> None:
        self.gltf.buffers.append(Buffer(byteLength=len(self._blob)))
        self.gltf.set_binary_blob(bytes(self._blob))
        self.gltf.save_binary(str(out_path))


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gothic-dir", required=True, type=Path, help="Gothic II install root (contains Data/*.vdf)")
    parser.add_argument("--world", default="NEWWORLD.ZEN", help="World file to export (default: NEWWORLD.ZEN)")
    parser.add_argument("--out", required=True, type=Path, help="Output .glb path")
    parser.add_argument("--world-mesh-only", action="store_true", help="Skip placed vobs, export only the static world mesh")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")

    data_dir = args.gothic_dir / "Data"
    vdf_files = sorted(set(data_dir.glob("*.vdf")) | set(data_dir.glob("*.VDF")))
    if not vdf_files:
        sys.exit(f"No .vdf archives found in {data_dir}")

    log.info("Mounting %d VDF archives from %s", len(vdf_files), data_dir)
    vfs = zenkit.Vfs()
    for vdf in vdf_files:
        log.debug("Mounting %s", vdf.name)
        vfs.mount_disk(vdf)

    zen_node = vfs.find(args.world)
    if zen_node is None:
        sys.exit(f"Could not find {args.world} in mounted VDFs")

    log.info("Loading world %s", args.world)
    world = zenkit.World.load(zen_node.open())

    tex_cache = TextureCache(vfs)
    builder = GlbBuilder()
    root_children: list[int] = []

    log.info("Building world mesh geometry")
    world_parts = build_mesh_parts_from_world_mesh(world.mesh, tex_cache)
    total_tris = sum(len(p.indices) // 3 for p in world_parts)
    textured = sum(1 for p in world_parts if p.material.texture_png is not None)
    log.info(
        "World mesh: %d material groups, %d triangles, %d with textures",
        len(world_parts),
        total_tris,
        textured,
    )
    world_node = SceneNode(name="WorldMesh", translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0), parts=world_parts)
    builder.add_node(world_node, root_children)

    if not args.world_mesh_only:
        log.info("Walking %d root vobs", len(world.root_objects))
        cache: dict = {}
        stats = {"vobs_with_mesh": 0}
        vob_nodes = walk_vobs(vfs, world.root_objects, cache, stats, tex_cache)
        log.info("Placed %d vob nodes with mesh geometry", stats["vobs_with_mesh"])
        vobs_root = SceneNode(name="Vobs", translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0), children=vob_nodes)
        builder.add_node(vobs_root, root_children)

    builder.gltf.scenes[0].nodes = root_children
    args.out.parent.mkdir(parents=True, exist_ok=True)
    builder.finalize(args.out)
    log.info("Wrote %s", args.out)


if __name__ == "__main__":
    main()
