# Knowledge Base — Agent Instructions

How to **find**, **write**, **link**, **tag** and **retire** notes in
`docs/knowledge/`. This file is the specification. [TAGS.md](./TAGS.md) is
the tag vocabulary, and `tools/kb.py` checks the rules and searches notes.
`CLAUDE.md` in this folder is a symlink to this file, so Claude Code loads
it automatically when it touches anything here.

Contents: [1 Model](#1-the-model-a-zettelkasten-for-agents) ·
[2 Finding](#2-finding-knowledge) · [3 When to write](#3-when-to-write-a-note) ·
[4 Note format](#4-note-format) · [5 Frontmatter](#5-frontmatter) ·
[6 Tags](#tags) · [7 Links](#7-links) · [8 INDEX hierarchy](#8-the-index-hierarchy) ·
[9 Maintenance](#9-maintenance-updating-superseding-archiving-deleting) ·
[10 Checklist](#10-checklist-before-you-commit)

## 1. The model: a Zettelkasten for agents

The knowledge base is the project's long-term memory. It holds what a future
agent needs but cannot get from the code, `git log`, or the progress logs:
why a design is the way it is, what was measured, what failed, which trap
cost a day, and what outside research says. It follows the Zettelkasten
method, with changes that make it work for agents that start every session
with an empty context.

| Zettelkasten principle | What it means here |
|---|---|
| **Atomic notes** — one idea per note | One concept, decision, lesson or reference topic per file. If you need "and" to describe the note, split it. |
| **Permanent notes stand alone** | A note must make sense when it is the *only* file an agent reads. Define terms or link the note that does. Never write "as discussed above" across files. |
| **Own words, not copies** | Distill. Cite sources (`sources:`), but write the claim, the numbers and the consequence for migera. |
| **Fixed address** | The path relative to `docs/knowledge/` is the note's ID. Renaming it means updating every inbound link (`kb.py backlinks`). Rename rarely. |
| **Links carry context** | A link without "why follow it" adds nothing. Every link says *when* or *why* to open it ([§7](#7-links)). |
| **Structure notes / MOCs** | Every folder's `INDEX.md` is a hand-curated structure note ([§8](#8-the-index-hierarchy)). |
| **Fleeting → permanent** | Scratch findings go in the session scratchpad or a progress log. Only distilled, verified knowledge becomes a note. |

Agents read with **progressive disclosure**:
1. The frontmatter `description` (one or two sentences).
2. The note body.
3. Its linked notes.

That is why `description` is the most important line in a note. It is what
`kb.py catalog`, the INDEX tables and grep hits show. An agent decides from
it alone whether to open the file.

**Only durable knowledge belongs here.** It must stay true and useful long
after the session that produced it ([§3](#the-durability-test)).

**What does not belong here:**
- Transient working state: plans for this session, to-dos, in-flight status.
- Chronological progress. That goes in `PROGRESS.md` and `CHARACTER_PROGRESS.md`.
- Instructions for how to operate in the repo. Those go in the root `AGENTS.md`.
- Anything derivable from the code itself.
- Per-user preferences.
- Raw research dumps. Distill them first.
- Book PDFs. Those go in `docs/books/`; link to them.

## 2. Finding knowledge

Do this **before** you design, debug a subsystem, or add a note. An hour of
rediscovery is the failure this knowledge base exists to prevent.

1. **Browse.** Start at [INDEX.md](./INDEX.md), go to the domain `INDEX.md`,
   then the topic `INDEX.md`, then the note. Each row's *Read when* column
   tells you whether to go deeper. Stop as soon as the rows stop matching
   your task.
2. **Catalog.** `python3 tools/kb.py catalog [SUBDIR]` prints every note's
   path, status and description in one screen. Grep that output.
3. **By tag.** `python3 tools/kb.py find ik locomotion` (all tags must
   match) or `... find ik ragdoll --any` (any tag matches). Add
   `--type lesson` or `--status current` to filter. Tag list and counts:
   `python3 tools/kb.py tags`.
4. **By code path.** `python3 tools/kb.py code src/character/anim/legik.rs`
   lists the notes that declare they describe that file. **Run it before
   changing a file.** Those notes hold the traps and decisions for it.
5. **Full text.** `rg -il 'foot lock' docs/knowledge` finds a term. Then read
   the frontmatter of the hits (`rg -A12 '^---' file`) before you read whole
   files.
6. **Backlinks.** `python3 tools/kb.py backlinks sdf-3d/rendering/sphere-tracing.md`
   shows what builds on a note.

**Trust levels.** Check `status` before you rely on a note:
- `current`: rely on it. If it also has a `verified` date, it was checked
  against the code or source on that date.
- `draft`: incomplete.
- `stale`: known to be out of date. Read it, but re-check its claims.
- `superseded`: follow `superseded_by` instead.
- `archived`: history only. Never base a decision on it.

A note older than the code it describes is a hint, not a fact. Re-verify
anything load-bearing. `kb.py stale` lists notes whose `code:` files changed
after the note's `verified` date (or its `updated` date if it has none).

## 3. When to write a note

Write or update a note when the knowledge would change what a future agent
does. Triggers:

| Trigger | Note `type` |
|---|---|
| A bug whose root cause was non-obvious, far from its symptom, or passed the test suite | `lesson` |
| A testing/verification/debugging trap: a test that couldn't fail, a misleading measurement | `lesson` |
| A design choice with real alternatives (chose X over Y because Z) | `decision` |
| A measured performance result, **including null results** ("tried X, zero win") | `lesson` or `decision` |
| An idea researched and deliberately *not* built, and why | `decision` |
| External research: papers, other engines, source-verified library internals | `research` / `reference` |
| An explanation of how a technique or subsystem works | `concept` |
| A step-by-step procedure or symptom→fix map | `guide` |
| A proposal or plan not yet built | `design` |

### The durability test

Only knowledge with **long-term value** goes in. Before you write a note, ask:
*"Will this still be true, and still change what an agent does, months from
now, after the code around it has moved on?"* If it won't, it doesn't belong
here.

| Durable: keep it | Transient: leave it out |
|---|---|
| Why something is designed the way it is, and what lost | What you're doing this session, next steps, TODO lists |
| A trap and the rule that avoids it | Step-by-step narration of a debugging session |
| Measured results, with how they were measured | Status of in-flight work: "half done", "blocked on X" |
| How a technique or library internal works | Values that change every commit: line numbers, counts that drift, today's test total |
| Dead ends, and why they are dead | Workarounds for a bug fixed in the same change |
| Invariants and contracts between subsystems | Branch names, PR numbers, anything only a git log reader needs |

Transient information still has a home:
- The session scratchpad, for working notes.
- The commit message, for what changed and why.
- `PROGRESS.md` and `CHARACTER_PROGRESS.md`, for chronology.
- Code comments, for local "why" that only matters next to the code.

**Distill, then file.** A debugging session can produce ten transient
observations and one durable lesson. Only the lesson becomes a note. If a
note has a transient part, strip it out before committing. A short
**Evidence** line (commit, test name, measurement) is the right amount of
history.

**Prune on contact.** When you open a note and find content that has gone
transient, delete it or fold it into the durable claim. Examples: a finished
plan's to-do list, or numbers from code that no longer exists. If the whole
note no longer passes the durability test, archive or delete it
([§9](#9-maintenance-updating-superseding-archiving-deleting)).

**Don't write** a note for:
- A routine fix. The commit message is enough.
- Something already in a note. **Update that note instead.**
- Speculation you did not verify. Label unverified claims or leave them out.

**Before you create a note, search** ([§2](#2-finding-knowledge)).
1. If a note covers the idea, extend or correct it.
2. If a note covers a neighbouring idea, create the new note and link the two
   both ways.
3. Only a genuinely new idea gets a new file.

## 4. Note format

**File name:** lower-kebab-case and descriptive. The name is the ID and a
grep target (`two-bone-ik-pivots-at-upper-not-root.md`, not `ik-notes.md`).

**Title** (`# H1`, the same as `title:`):
- Lessons and decisions: a **claim** that states the conclusion ("PD damping
  has an explicit-integration bound").
- Concepts and references: a noun phrase ("Sphere tracing").

**Size:**
- Aim for 40–250 lines. `lint` warns at 300.
- Over 150 lines, put a one-line `Contents:` list of section links under the H1.
- If a note keeps growing, split out sub-ideas as their own notes. Leave a
  short overview that links to them.

**Body skeletons.** Adapt them, but keep the order: claim first, evidence
after.

```markdown
# <Claim or concept name>

<1–3 sentences: the whole point. A reader who stops here still got it.>

## <type-specific sections, see below>

## Related
- [Other note](../x/other-note.md) — <relation>: <when/why to open it>
```

| type | Sections after the opening claim |
|---|---|
| `lesson` | **What happened** (concrete, with numbers) · **Why it matters** (the general trap) · **How to apply** (the rule to follow next time) · **Evidence** (commits, test names, measurements) |
| `decision` | **Context** · **Decision** · **Alternatives considered** (with why each lost, measured if possible) · **Consequences** · **Revisit when** |
| `concept` / `research` / `reference` | Free structure. Start with the key facts. Put sources in `sources:` and cite inline where a number comes from. End with **Relevance to migera** if the note is external research. |
| `guide` | **When to use** · numbered steps or a symptom → cause → fix table |
| `design` | **Goal** · **Design** · **Open questions** · **Status** (what's built, what isn't) |

**Writing rules:**
- Absolute dates (`2026-09-28`), never "recently" or "last week".
- Real measured numbers with units and how they were measured (for example
  `--bench 10`, or `anim_bench --characters 100`), never "much faster".
- Name code precisely: `src/character/anim/legik.rs`, `fn solve_two_bone`,
  or a test name. Also list the paths in `code:`.
- One term per concept across the knowledge base. If a note uses a synonym,
  add it to `aliases:` so grep finds it.

## 5. Frontmatter

Every `.md` file under `docs/knowledge/` has YAML frontmatter, INDEX files
included. The exception is the meta files `AGENTS.md`, `CLAUDE.md` and
`TAGS.md`. Use exactly this shape: scalar values on one line, lists as block
lists with two-space `- ` items.

```yaml
---
title: Two-bone IK pivots at the upper joint, not the root
description: >-
  Leg IK must measure reach from the upper-leg joint, not the hip socket;
  measuring from the socket lands short by the socket offset. Read before
  touching legik.rs or writing an IK reach test.
type: lesson
status: current
tags:
  - ik
  - correctness
  - testing
updated: 2026-09-20
verified: 2026-09-20
code:
  - src/character/anim/legik.rs
sources:
  - commit 4c7fbc9
aliases:
  - leg IK reach shortfall
---
```

(`>-` folded scalars are allowed for `description` only. `kb.py` reads
`description` on one line, so prefer a single line when it fits.)

| Field | Req. | Meaning |
|---|---|---|
| `title` | yes | Same as the H1. |
| `description` | yes | ≤ 320 chars, third person. It says **what the note establishes** and **when to read it** ("Read before …", "Read when …"). Write it for someone deciding whether to open the file. Not "Notes about IK". |
| `type` | yes | `index` · `concept` · `reference` · `guide` · `research` · `design` · `decision` · `lesson` |
| `status` | yes | `current` · `draft` · `stale` · `superseded` · `archived` ([§9](#9-maintenance-updating-superseding-archiving-deleting)) |
| `tags` | yes | 2–6 tags from [TAGS.md](./TAGS.md) ([§6](#tags)). INDEX files carry their domain's main tags. |
| `updated` | yes | `YYYY-MM-DD` of the last *content* change. Formatting-only edits don't count. |
| `verified` | no | `YYYY-MM-DD` when the claims were last checked against the code or the primary source. |
| `created` | no | `YYYY-MM-DD` first written. |
| `code` | no | Repo-relative paths (files or dirs) this note describes. It powers `kb.py code` and `kb.py stale`. Set it on every project-specific note. |
| `sources` | no | Papers, URLs, book + page, commits, test names. |
| `superseded_by` | if superseded | Relative path to the replacement note. |
| `aliases` | no | Synonyms and old names, so grep finds the note under the words people search. |

<a id="tags"></a>
## 6. Tags

- Use tags only from [TAGS.md](./TAGS.md). `lint` rejects any other tag.
- Tags are **facets that cut across folders**: `performance`, `correctness`,
  `testing`, `ik`, `raymarching`… Don't repeat the folder as a tag unless a
  note elsewhere would want the same tag. For example, `sdf` on a
  `3dgs/` note that converts SDFs is right.
- Use 2–6 tags per note. Always include at least one **concern** tag when one
  applies (`performance`, `correctness`, `testing`, `debugging`,
  `verification`, `numerics`). "Every lesson about a test that couldn't
  fail" must be one query away.
- **New tag:** only when at least three notes would use it. Add a row to
  TAGS.md in the same commit. **Merging tags:** retag every user
  (`kb.py find old`), then delete the row.
- Specific terms (`ggx`, `fxaa`, `colmap`) are *not* tags. Full-text search
  finds them in the body. If the body doesn't use the term people will
  search for, add it to `aliases:`.

## 7. Links

**Format:** relative Markdown links only, `[Readable title](../topic/note.md)`,
optionally with `#anchor`. They render on GitHub, an agent can open the path
directly, and `lint` checks them. **No `[[wikilinks]]`**: they don't render
and can't be checked. Link to source code with a repo-relative path from the
note (`../../../src/…`) or as inline code (`src/character/anim/legik.rs`).

**Every link says why to follow it.** Two forms:
- **Inline**, where the sentence itself gives the reason: "…which is why the
  step must be damped (see [exact vs. bound SDFs](../fundamentals/exact-vs-bound-sdfs.md))."
- **`## Related` list** at the end of the note, one link per line:
  `- [Title](path) — <relation>: <when/why to open it>`

  Relations: **prerequisite** (read first) · **deeper** (detail on a point
  made here) · **applies** (where this is used in migera) · **example**
  (a concrete case of this rule) · **contrast** (competing approach or
  counter-evidence) · **supersedes** / **superseded-by** · **same-trap**
  (the same failure pattern in another subsystem).

  Example: `- [Same function both sides is a vacuous test](../engineering-practice/same-function-both-sides-is-a-vacuous-test.md) — same-trap: read when a comparison test passes suspiciously easily.`

**Link both ways** when the relation is meaningful from both ends. When you
write note B that refines note A, add B to A's `Related` too. Lessons should
link to the concept notes they correct, and concept notes should link back
to their lessons. That backlink is how the next agent reading the concept
learns about the trap.

Keep chains shallow. Everything an agent needs to act on a note should be at
most **one hop** from it. Don't make it read A → B → C to find a caveat that
belongs in A. A one-sentence summary plus a link is better than a bare link.

## 8. The INDEX hierarchy

Every folder has an `INDEX.md` structure note. There are at most three
levels:

```
INDEX.md                    root: map of domains (always small)
└── <domain>/INDEX.md       domain overview: what the domain is, key facts, topics
    └── <topic>/INDEX.md    topic: its notes, reading order
        └── <note>.md
```

A small domain can hold notes directly without topic folders. Don't nest
deeper than `domain/topic/note.md`. If a topic outgrows ~15 notes, split it
into sibling topics, not a deeper subfolder.

**Exception: book digests.** A domain that digests one book mirrors the
book's own structure, one level deeper than usual:
`<book>/INDEX.md` (whole-book summary) → `chNN-<slug>/INDEX.md` (chapter
summary) → `N.M-<slug>/INDEX.md` (section summary) → `N.M.K-<slug>.md`
(subsection note). A section without subsections is a note
(`N.M-<slug>.md`) in its chapter folder. Here each INDEX is also the
summary of its unit, so it may carry synthesized content beyond routing.
Every file cites its printed pages and links the PDF page (`…pdf#page=N`)
so a reader can restore the full context. Example:
[biomechanics-winter](./biomechanics-winter/INDEX.md).

**Rules for an INDEX:**
- Frontmatter `type: index` and a `description` of the whole subtree.
- It lists **every** note and child INDEX in its folder, with no omissions
  (`lint` checks). It may also link to related notes elsewhere, under
  "See also".
- Each entry is a table row: `| [Title](./note.md) | What it establishes | Read when |`.
  The *Read when* cell names a concrete trigger ("before changing
  `legik.rs`", "when shadows band"), not "for more info".
- Optional **Start here / reading order** when order matters.
- Optional **Key facts**: at most 10 bullets, each a one-line claim with a
  link to the note that proves it. This is the domain in ninety seconds.
- Notes with status `archived` or `superseded` go in a final
  **Archived / superseded** table. A reader skipping history never has to
  wade through them.
- An INDEX summarizes and routes. Don't put knowledge only in an INDEX. If a
  "key fact" has no note behind it, write the note.

**When you add, move, rename or retire a note, update its INDEX in the same
change.** Update the parent INDEX's row too if the subtree's scope changed.
The root [INDEX.md](./INDEX.md) changes only when a domain is added or
retired.

## 9. Maintenance: updating, superseding, archiving, deleting

Knowledge decays. Keeping it true matters as much as adding to it.

**Update in place** when a fact changes, a number is re-measured, or a
caveat is found. Bump `updated` and set `verified` if you checked it against
the code. Rewrite the claim so it is true now. Don't append "UPDATE:"
paragraphs that contradict the text above them. If the history matters, add
one line: "Was X until commit abc123 (2026-08-01); see …".

**Mark `stale`** when you notice code has moved past a note but you can't fix
the note now. Add a one-line `> **Stale:** <what changed, date>` under the
H1. A stale note is a to-do, not a resting state.

**Supersede** when a new note replaces an old one's conclusion (a decision
reversed, a better technique adopted):
1. New note: add a `supersedes` link in Related.
2. Old note: `status: superseded`, `superseded_by: <path>`, and a one-line
   banner under the H1 saying what replaced it and why.
3. Move the old note's INDEX row to *Archived / superseded*.
4. `kb.py backlinks old.md`, then repoint the links that meant "the current
   answer".

**Archive** (`status: archived`, banner, row moves to *Archived*) when the
subject itself is gone but the record is still worth having. Examples: a
deleted module, or a retired approach with its measured reasons.
Dead-end records are valuable. They stop the next agent from retrying them.

**Delete** only when a note has **no remaining value**: it is fully merged
into another note, or it is wrong and has no history worth keeping. Git
keeps the old text. Before deleting, repoint every backlink
(`kb.py backlinks`) and remove the INDEX row. **Merge** duplicates the same
way: fold the content into the better-named note, add the loser's name to
its `aliases`, then delete the loser.

**Periodic hygiene.** Do this whenever you work in a domain for more than a
quick lookup:
- `kb.py stale`: re-verify or mark stale.
- `kb.py lint`: fix errors, then look at the warnings (isolated notes,
  oversized notes, vanished `code:` paths).
- If an INDEX's *Key facts* contradict its notes, fix the INDEX.

## 10. Checklist before you commit

- [ ] Passes the durability test ([§3](#the-durability-test)): long-term
      value only, with transient session detail stripped out.
- [ ] Searched first. This is a new idea, or an existing note was updated.
- [ ] Frontmatter complete. `description` says what and when. Tags come
      from TAGS.md. `code:` is set for project notes.
- [ ] Claim first. Numbers measured, dates absolute, sources cited.
- [ ] Each link says why. Related notes link back.
- [ ] Parent `INDEX.md` row added or updated.
- [ ] `python3 tools/kb.py lint` reports 0 errors.
