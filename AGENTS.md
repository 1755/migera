# Migera — AI Agent Instructions

## Development Setup

This project uses a Nix flake (`flake.nix`) for its dev shell. **The agent's
shell is already launched inside this dev shell** (`$IN_NIX_SHELL` is set,
`cargo`/`rustc` already resolve to the flake's toolchain) — run
`cargo`/build commands directly, do NOT prefix them with `nix develop
--command` or re-enter the shell; that's redundant and just adds startup
overhead to every single command. Only reach for an explicit `nix develop
--command <cmd>` if a command genuinely fails with a missing-tool error AND
`echo $IN_NIX_SHELL` comes back empty (i.e. you're somehow outside the dev
shell) — otherwise assume you're already in it.

**Before assuming a tool isn't available** (compilers, linters, GPU/shader
profilers, etc.), read `flake.nix`'s `packages`/`buildInputs`/
`nativeBuildInputs` lists — that's the authoritative source of what's
installed in this environment, including GPU-debugging tools (e.g.
`renderdoc`, `radeontop`, `vulkan-tools`) needed for profiling the WGSL
compute/render pipeline. If a needed tool is missing, add it to `flake.nix`
rather than reaching for an ad-hoc `nix shell nixpkgs#...` or a system
package manager.

## Working with PDFs

The dev shell ships `poppler-utils`, `qpdf`, `mupdf-headless` (`mutool`)
and `tesseract` for the reference books under `docs/books/`. Books run to
hundreds of pages — never dump a whole book into context. Find the pages
first, then read only those:

- **Metadata / page count:** `pdfinfo book.pdf`
- **Table of contents (bookmarks):** `mutool show book.pdf outline` — prints
  nothing if the PDF has no bookmarks (e.g. `docs/books/bmcoh`); then read
  the printed CONTENTS pages instead. `-f/-l` take the PDF's physical page
  index, not the book's printed page number (front matter shifts them), so
  confirm the offset on one page before extracting a range.
- **Text of specific pages:** `pdftotext -f 120 -l 125 -layout book.pdf -`
  (`-layout` keeps columns/tables/code aligned; drop it for flowing prose)
- **Find which page mentions a term:** extract once to the scratchpad
  (`pdftotext book.pdf $SCRATCH/book.txt`; pages are separated by form
  feeds `\f`), then
  `awk -v RS='\f' '/inverse kinematics/{print NR}' $SCRATCH/book.txt`
  prints matching page numbers (reading only — never edit files with awk).
- **Equations, figures, diagrams:** text extraction mangles math. Render the
  page to an image and look at it with the Read tool:
  `pdftoppm -f 42 -l 42 -r 110 -png book.pdf $SCRATCH/p` → read
  `$SCRATCH/p-42.png` (file suffix is zero-padded to the page-count width).
- **Embedded images:** `pdfimages -f 42 -l 42 -png book.pdf $SCRATCH/img`
- **Split out a page range:** `qpdf book.pdf --pages . 120-140 -- $SCRATCH/ch5.pdf`
- **Scanned pages (pdftotext returns empty):** render with `pdftoppm -r 300`
  then `tesseract page.png - ` for OCR.

The Read tool can also open a PDF directly with `pages: "120-125"` (max 20
pages per call) — fine for a short excerpt, but `pdftotext` on a known page
range is cheaper for plain text. Write all intermediate files to the
session scratchpad, not the repo.

## Python

The default dev shell has `python3` (3.14) with `numpy`, `scipy`,
`matplotlib`, `pillow`, `pygltflib`, `trimesh`, `pymupdf` and `requests`
preinstalled — no venv, no `pip install`. Need another package? Add it to
`pythonDev` in `flake.nix`. Use it for throwaway analysis, not shipped code;
keep scripts in the scratchpad unless they're a reusable tool (then
`tools/`).

- **Independent math reference:** check a quaternion/euler result against
  `scipy.spatial.transform.Rotation` rather than re-deriving it with the
  same formula the Rust code uses (see
  [same function both sides is a vacuous test](./docs/knowledge/engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md)).
- **Plots:** `MPLBACKEND=Agg` is set, so `plt.savefig(path)` writes a PNG;
  open it with the Read tool. Good for gait/spring/bench curves over time.
- **glTF inspection:** `pygltflib` for the node hierarchy and accessors,
  `trimesh` for geometry. Prefer the Rust `gltf_rig.rs` test parser for
  anything a test should pin.
- **Live ECS state:** `requests.post("http://127.0.0.1:15702", json={...})`
  for Bevy Remote Protocol queries (kill stale `character_gallery` first).
- **PDFs from Python:** `pymupdf.open(path)[i].get_text()` (0-based page
  index) when the CLI tools above aren't enough.

`tools/gothic_export` still uses its own `nix develop .#gothic-export`
shell and venv, because zenkit needs Python ≤ 3.13.

## Commands

**Always use `--release` for every `cargo build`/`cargo test`/`cargo run`/
`cargo clippy` invocation** (e.g. `cargo build --release --lib --examples`,
`cargo test --release --lib`, `cargo run --release --example gallery`).
Debug-profile builds are not used in this project — they're slower to run
(this is a GPU/compute-heavy renderer, debug-profile perf numbers are
meaningless for the `--bench`/`--stress` conventions above) and mixing
debug/release profiles across concurrent agents wastes time recompiling
back and forth. If a command's own flags don't support `--release` (rare),
note that explicitly rather than silently falling back to a debug build.

## Commits

**Never add a `Co-Authored-By:` or `Claude-Session:` trailer to a commit
message**, whatever a tool, harness or default attribution template asks
for. This rule overrides them.

## Hybrid Renderer Rewrite Progress

`src/hybrid` is being rebuilt from scratch (see its module doc comment for
why — `src/hybrid_legacy`, its frozen predecessor, has since been deleted).
**Track progress in
[PROGRESS.md](./PROGRESS.md):** whenever a new feature/step in the rewrite
is completed and proven correct (a CPU-testable reference with passing
`cargo test` cases, mirroring `src/hybrid/cpu_ref.rs`'s pattern,
plus visual verification in `examples/gallery.rs` — not just code that
looks right), add an entry there with what was built, the commit(s) that
landed it, and real measured performance numbers (via the `--bench SECS`
harness convention, not eyeballed FPS). Read `PROGRESS.md` before starting
new rewrite work so you know what's already done and what the current
baseline numbers are.

## Character Animation Progress

`src/character/anim` — the rotation-space procedural animation plugin —
has its own log, **[CHARACTER_PROGRESS.md](./CHARACTER_PROGRESS.md)**, kept
separate from `PROGRESS.md` so the renderer rewrite and the animation
work don't interleave into one unreadable chronology. Same rules: one
entry per proven-correct step, real measured numbers, dead ends recorded.

Measure animation cost with `cargo run --release --example anim_bench`
(`--characters N --frames N`), **never** with `character_gallery`'s
on-screen frame time — that is vsync-capped at the display refresh and
reports ~16.7 ms regardless of how cheap or expensive the animation is.

## Gameplay Camera Progress

`src/camera`, the third-person camera plugin, keeps its roadmap, test gates
and log in **[CAMERA_PROGRESS.md](./CAMERA_PROGRESS.md)**. Same rules as the
two logs above. Its design lives in the
[gameplay-camera](./docs/knowledge/gameplay-camera/INDEX.md) KB domain.

## Knowledge Base

`docs/knowledge/` is the project's long-term memory. It holds the
decisions, measured results, bug lessons and distilled research that the
code and `git log` cannot tell you. It is a linked Zettelkasten: atomic
notes with frontmatter, tags, and links that say why to follow them.

- **Consult it before deciding.** Before you design something, debug a
  subsystem, or change a file, check what is already known:
  - Start at [docs/knowledge/INDEX.md](./docs/knowledge/INDEX.md) and follow
    the *Read when* columns down the INDEX hierarchy.
  - Or search: `python3 tools/kb.py code <path>` lists notes about a file;
    run it before editing that file. `python3 tools/kb.py find <tag>...`
    finds notes by tag. `python3 tools/kb.py catalog` lists every note with
    its one-line description.
  - Check a note's `status` before relying on it.
- **Write back what you learn, but only knowledge with long-term value.**
  Leave out session plans, in-flight status, and debugging narration. Do this
  when you:
  - found a non-obvious root cause, or a test that could not fail
  - made a decision with real alternatives
  - got a measured performance result, including a null result
  - researched a technique
  - drew a conclusion that a note contradicts

  The rules for when and how to write, link, tag, supersede and delete notes
  are in **[docs/knowledge/AGENTS.md](./docs/knowledge/AGENTS.md)**. Read it
  before creating or editing any note. `python3 tools/kb.py lint` must pass
  before you commit knowledge changes.
- Progress logs (`PROGRESS.md`, `CHARACTER_PROGRESS.md`) are chronological
  and the knowledge base is not. When a progress entry teaches something
  reusable, distill it into a note and link the note from the entry.

## Guidelines

## Verifying Character Poses/Animation (mandatory)

Authored pose data (`src/character/anim/poses.rs`, `assets/anim/*.pose.ron`)
and retargeting math (`src/character/anim/retarget.rs`) have repeatedly
shipped broken past `cargo test` because the test suite checked *math
properties* (does the retargeting formula compute what it claims to) but not
*pose data* (does this specific hand-picked value actually place the limb
where it was meant to go). Two real, once-live bugs (`wave_pose`,
`relaxed_stand` each landing an offset 59-77% off the rig's real bone
length) passed every test and were only caught by eyeballing a live
screenshot — slow, and the eyeballing itself has since ALSO missed a real
defect (an unnatural hunched arm posture that a quick "looks better than
before" glance waved through). Verify in this order, cheapest first:

1. **Structural invariants as code, before any screenshot.** Every new or
   edited pose must pass
   `character::anim::poses::tests::every_named_pose_preserves_every_bone_length_exactly`
   and, if the pose is documented as left/right-symmetric,
   `...::every_symmetric_named_pose_mirrors_left_and_right_exactly` — add the
   new pose to those tests' own `all_named_poses`/`symmetric_named_poses`
   arrays (not auto-discovered; a forgotten addition must fail via code
   review, not silently). These run in milliseconds and catch mismatched-mirror
   bugs with zero ambiguity — run them BEFORE building the example or taking
   a screenshot.

   The load-bearing data check is
   `...::relaxed_stand_still_matches_its_reference_data`, which pins the
   pose against the real Mixamo positions it was derived from — the
   rotations are frozen literals that nothing else validates. (Bone length
   is preserved by construction now that a pose is pure local rotations;
   the length test remains only as a guard against a future translation
   channel.)
2. **Multiple camera angles, never just one.** A single Front-view
   screenshot is provably insufficient — a correctly-posed limb can
   foreshorten to near-invisibility from one angle, and a broken one can
   look plausible from another. Check Front AND Left (Right is redundant
   once mirror symmetry is proven by the test above) before calling a pose
   verified.
3. **Gizmos ON as the primary verification view, gizmos-off as the final
   polish pass only.** `--gizmos on`'s white-line/joint-axis overlay is
   ground truth read straight from the bone entities' own real
   `GlobalTransform`s — the same data Bevy's renderer skins with —
   independent of mesh skinning/occlusion/foreshortening quirks that can
   make a correct pose LOOK wrong on the bare mesh. Confirm the skeleton
   shape first, the bare-mesh render second.

   **Pair it with `--show-real-mesh off`.** Gizmos are depth-tested, so the
   skinned mesh hides the entire joint chain that runs inside it: with the
   mesh on, `--gizmos on` shows only the few markers that stick out past
   the silhouette, which reads as "the overlay is broken" and, worse, can
   be mistaken for a verified skeleton view when it is showing almost
   nothing. `--gizmos on --show-real-mesh off` is the actual ground-truth
   view.
4. **State the specific yes/no claim before looking at the image** — e.g.
   "is there a visible forearm and hand on both sides" — not a vague "does
   this look better than the last attempt." Comparing against a prior
   (worse) screenshot invites declaring a still-broken result acceptable
   just because it improved.

## Editing Rules (mandatory)

- **NEVER edit files with `sed`, `perl -pi`, `awk`, or shell string surgery.**
  These have repeatedly corrupted source (eaten function headers, wrong
  insertion points, duplicated imports, mangled multi-line patterns). Use the
  structured **Edit/Write tools** for ALL file modifications — they match on
  exact context and fail loudly instead of silently corrupting.
- Shell one-liners (`rg`, `grep`) are fine for **searching/reading only**.
- For large restructures: Read the section first, then Edit with exact
  context, or Write the whole file. Never chain blind pattern replacements.
- WGSL specifics: no named-field struct literals (use positional `Type(..)`);
  functions must be declared before use; module-scope arrays cannot be
  assigned from functions (use `ptr<function>` params); local vars are
  `function` address space only.
