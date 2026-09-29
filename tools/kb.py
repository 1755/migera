#!/usr/bin/env python3
"""Knowledge-base tool for docs/knowledge — search, navigate and lint.

The rules this enforces are specified in docs/knowledge/AGENTS.md; the tag
vocabulary lives in docs/knowledge/TAGS.md. Stdlib only, so it runs from any
python3.

    python3 tools/kb.py catalog [SUBDIR]        path — description, one per note
    python3 tools/kb.py find TAG [TAG...] [--type T] [--status S] [--any]
    python3 tools/kb.py tags                    tag usage counts (and unregistered tags)
    python3 tools/kb.py backlinks PATH          notes that link to PATH
    python3 tools/kb.py code PATH               notes whose `code:` covers PATH
    python3 tools/kb.py stale                   notes whose `code:` changed after `verified`/`updated`
    python3 tools/kb.py lint                    check every rule; exit 1 on errors
"""

import os
import re
import subprocess
import sys
from collections import Counter, defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KB = os.path.join(REPO, "docs", "knowledge")

# Files in the KB root that are about the KB, not knowledge notes.
META_FILES = {"AGENTS.md", "CLAUDE.md", "TAGS.md"}

TYPES = {"index", "concept", "reference", "guide", "research", "design", "decision", "lesson"}
STATUSES = {"current", "draft", "stale", "superseded", "archived"}
REQUIRED = ["title", "description", "type", "status", "tags", "updated"]
DATE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
MAX_DESCRIPTION = 320
WARN_LINES = 300
LINK = re.compile(r"\]\(([^)\s]+)\)")
WIKILINK = re.compile(r"\[\[[^\]]+\]\]")


def parse_frontmatter(text):
    """Parse the YAML subset the KB uses: `key: scalar`, `key: [a, b]`, and
    `key:` followed by `  - item` lines. Returns (dict, body) or (None, text)."""
    if not text.startswith("---\n"):
        return None, text
    end = text.find("\n---\n", 4)
    if end < 0:
        return None, text
    meta, key, folded = {}, None, False
    for line in text[4:end].splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if line.startswith((" ", "\t")) and key is not None:
            item = line.strip()
            if folded:
                meta[key] = (meta[key] + " " + item).strip()
            elif item.startswith("- "):
                if not isinstance(meta.get(key), list):
                    meta[key] = []
                meta[key].append(unquote(item[2:].strip()))
            continue
        k, sep, v = line.partition(":")
        if not sep:
            continue
        key, v = k.strip(), v.strip()
        folded = v in (">", ">-", "|", "|-")
        if folded:
            meta[key] = ""
        elif v.startswith("[") and v.endswith("]"):
            meta[key] = [unquote(x.strip()) for x in v[1:-1].split(",") if x.strip()]
        else:
            meta[key] = unquote(v)
    return meta, text[end + 5:]


def unquote(s):
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "\"'":
        return s[1:-1]
    return s


def as_list(v):
    if v is None or v == "":
        return []
    return v if isinstance(v, list) else [v]


def rel(path):
    return os.path.relpath(path, KB)


def load():
    notes = {}
    for d, dirs, files in os.walk(KB):
        dirs[:] = sorted(x for x in dirs if not x.startswith("."))
        for f in sorted(files):
            if not f.endswith(".md"):
                continue
            p = os.path.join(d, f)
            if d == KB and f in META_FILES:
                continue
            text = open(p, encoding="utf-8").read()
            meta, body = parse_frontmatter(text)
            notes[rel(p)] = {"meta": meta or {}, "has_fm": meta is not None,
                             "text": text, "body": body, "abs": p}
    return notes


def registered_tags():
    path = os.path.join(KB, "TAGS.md")
    if not os.path.exists(path):
        return set()
    # A registered tag is a table row whose first cell is `tag` in backticks.
    return set(re.findall(r"^\|\s*`([a-z0-9][a-z0-9-]*)`\s*\|", open(path).read(), re.M))


def links_of(note_rel, text):
    """Yield (target_rel_or_None, raw) for every relative Markdown link."""
    base = os.path.dirname(os.path.join(KB, note_rel))
    for m in LINK.finditer(text):
        raw = m.group(1)
        target = raw.split("#")[0]
        if not target or re.match(r"^[a-z]+:", target):
            continue
        yield os.path.normpath(os.path.join(base, target)), raw


def cmd_catalog(args):
    notes = load()
    sub = args[0].rstrip("/") if args else ""
    for p, n in notes.items():
        if sub and not p.startswith(sub):
            continue
        m = n["meta"]
        flag = "" if m.get("status", "current") == "current" else f" [{m.get('status')}]"
        print(f"{p}{flag} — {m.get('description', '(no description)')}")


def cmd_find(args):
    want_type = want_status = None
    tags, any_mode, i = [], False, 0
    while i < len(args):
        if args[i] == "--type":
            want_type, i = args[i + 1], i + 2
        elif args[i] == "--status":
            want_status, i = args[i + 1], i + 2
        elif args[i] == "--any":
            any_mode, i = True, i + 1
        else:
            tags.append(args[i])
            i += 1
    for p, n in load().items():
        m = n["meta"]
        have = set(as_list(m.get("tags")))
        if tags and not (have & set(tags) if any_mode else set(tags) <= have):
            continue
        if want_type and m.get("type") != want_type:
            continue
        if want_status and m.get("status") != want_status:
            continue
        print(f"{p} [{m.get('type')}/{m.get('status')}] — {m.get('description', '')}")


def cmd_tags(_args):
    reg = registered_tags()
    c = Counter(t for n in load().values() for t in as_list(n["meta"].get("tags")))
    for t, k in c.most_common():
        print(f"{k:4d}  {t}{'' if t in reg else '   <-- NOT IN TAGS.md'}")
    unused = sorted(reg - set(c))
    if unused:
        print("registered but unused:", ", ".join(unused))


def cmd_backlinks(args):
    target = os.path.normpath(os.path.join(KB, args[0])) if not os.path.isabs(args[0]) else args[0]
    if not os.path.exists(target):
        target = os.path.normpath(os.path.abspath(args[0]))
    for p, n in load().items():
        for t, raw in links_of(p, n["text"]):
            if t == target:
                print(f"{p}  ->  {raw}")


def cmd_code(args):
    want = os.path.normpath(args[0])
    for p, n in load().items():
        for c in as_list(n["meta"].get("code")):
            c = os.path.normpath(c)
            if want == c or want.startswith(c + os.sep) or c.startswith(want + os.sep):
                print(f"{p} — {n['meta'].get('description', '')}")
                break


def git_date(path):
    out = subprocess.run(["git", "-C", REPO, "log", "-1", "--format=%cs", "--", path],
                         capture_output=True, text=True).stdout.strip()
    return out or None


def cmd_stale(_args):
    for p, n in load().items():
        m = n["meta"]
        if m.get("status") in ("archived", "superseded"):
            continue
        seen = m.get("verified") or m.get("updated")
        if not seen:
            continue
        for c in as_list(m.get("code")):
            d = git_date(c)
            if d and d > seen:
                print(f"{p}: `{c}` changed {d}, note last checked {seen}")


def cmd_lint(_args):
    notes = load()
    reg = registered_tags()
    errors, warns = [], []
    E = lambda p, msg: errors.append(f"{p}: {msg}")
    W = lambda p, msg: warns.append(f"{p}: {msg}")
    if not reg:
        errors.append("TAGS.md: no registered tags found")

    inbound = defaultdict(set)
    for p, n in notes.items():
        m = n["meta"]
        for t, raw in links_of(p, n["text"]):
            if not os.path.exists(t):
                E(p, f"broken link `{raw}`")
            elif t.startswith(KB):
                inbound[rel(t)].add(p)
        if not n["has_fm"]:
            E(p, "missing frontmatter")
            continue
        for k in REQUIRED:
            if not m.get(k):
                E(p, f"missing required field `{k}`")
        if m.get("type") and m["type"] not in TYPES:
            E(p, f"unknown type `{m['type']}`")
        if m.get("status") and m["status"] not in STATUSES:
            E(p, f"unknown status `{m['status']}`")
        for k in ("updated", "verified", "created"):
            if m.get(k) and not DATE.match(m[k]):
                E(p, f"`{k}` is not YYYY-MM-DD")
        is_index = os.path.basename(p) == "INDEX.md"
        if is_index != (m.get("type") == "index"):
            E(p, "INDEX.md files (and only they) have type: index")
        tags = as_list(m.get("tags"))
        for t in tags:
            if t not in reg:
                E(p, f"tag `{t}` not registered in TAGS.md")
        if not is_index and not 1 <= len(tags) <= 8:
            W(p, f"{len(tags)} tags (aim for 2-6)")
        desc = m.get("description", "")
        if len(desc) > MAX_DESCRIPTION:
            W(p, f"description is {len(desc)} chars (max {MAX_DESCRIPTION})")
        if m.get("status") == "superseded":
            sb = m.get("superseded_by")
            if not sb:
                E(p, "status superseded without `superseded_by`")
            elif not os.path.exists(os.path.normpath(os.path.join(os.path.dirname(n["abs"]), sb))):
                E(p, f"superseded_by `{sb}` does not exist")
        for c in as_list(m.get("code")):
            if not os.path.exists(os.path.join(REPO, c)):
                W(p, f"code path `{c}` no longer exists — note may be stale")
        if WIKILINK.search(n["body"]):
            E(p, "contains [[wikilinks]] — use relative Markdown links")
        lines = n["text"].count("\n")
        if not is_index and lines > WARN_LINES:
            W(p, f"{lines} lines — consider splitting into atomic notes")

    # Hierarchy: every folder has an INDEX.md, and each note/sub-INDEX is
    # linked from its own folder's (resp. parent folder's) INDEX.md.
    dirs = {os.path.dirname(p) for p in notes}
    for d in sorted(dirs):
        if os.path.join(d, "INDEX.md") not in notes:
            E(d or ".", "folder has no INDEX.md")
    for p in notes:
        d = os.path.dirname(p)
        if os.path.basename(p) == "INDEX.md":
            if not d:
                continue
            parent = os.path.join(os.path.dirname(d), "INDEX.md")
        else:
            parent = os.path.join(d, "INDEX.md")
        if parent in notes and parent not in inbound[p]:
            E(p, f"not listed in its parent `{parent}`")
    for p in notes:
        if os.path.basename(p) != "INDEX.md" and not (inbound[p] - {os.path.join(os.path.dirname(p), "INDEX.md")}):
            if not any(True for _ in links_of(p, notes[p]["body"])):
                W(p, "isolated: no links in or out besides its INDEX — add Related links")

    for w in warns:
        print("warn:", w)
    for e in errors:
        print("ERROR:", e)
    print(f"{len(notes)} files, {len(errors)} errors, {len(warns)} warnings")
    return 1 if errors else 0


COMMANDS = {"catalog": cmd_catalog, "find": cmd_find, "tags": cmd_tags,
            "backlinks": cmd_backlinks, "code": cmd_code, "stale": cmd_stale, "lint": cmd_lint}

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in COMMANDS:
        print(__doc__)
        sys.exit(2)
    sys.exit(COMMANDS[sys.argv[1]](sys.argv[2:]) or 0)
