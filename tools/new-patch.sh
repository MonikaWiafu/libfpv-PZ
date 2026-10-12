#!/usr/bin/env bash
# Captures the uncommitted changes of a patched submodule that are not yet in
# its patch series as a new patch file.
#   tools/new-patch.sh simitl 0003-short-name
#   tools/new-patch.sh betaflight 0001-short-name
# The submodule working tree is expected to contain all existing patches
# plus the new changes (CRLF endings are preserved).
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
name="$1"; title="$2"
case "$name" in
  simitl) sub="$root/third_party/SimITL" ;;
  betaflight) sub="$root/third_party/betaflight" ;;
  *) echo "unknown submodule $name" >&2; exit 1 ;;
esac
series="$root/patches/$name"
tmp="$(mktemp -d)"
trap 'git -C "$sub" worktree remove --force "$tmp/wt" >/dev/null 2>&1 || true; rm -rf "$tmp"' EXIT
git -C "$sub" worktree add -q --detach "$tmp/wt" HEAD
for p in $(ls "$series"/*.patch 2>/dev/null | sort); do
  git -C "$tmp/wt" apply --whitespace=nowarn "$p"
done
out="$series/$title.patch"
: > "$out"
# changed tracked files plus untracked files
files="$( (git -C "$sub" diff --name-only; git -C "$sub" ls-files --others --exclude-standard) | sort -u)"
for f in $files; do
  if [ -e "$tmp/wt/$f" ]; then
    if ! cmp -s "$tmp/wt/$f" "$sub/$f"; then
      { echo "diff --git a/$f b/$f"; diff -u --label "a/$f" --label "b/$f" "$tmp/wt/$f" "$sub/$f" || true; } >> "$out"
    fi
  else
    { echo "diff --git a/$f b/$f"; echo "new file mode 100644"; diff -u --label /dev/null --label "b/$f" /dev/null "$sub/$f" || true; } >> "$out"
  fi
done
if [ ! -s "$out" ]; then rm "$out"; echo "no new changes"; exit 0; fi
# verify: the whole series must reproduce the working tree
git -C "$tmp/wt" apply --whitespace=nowarn "$out"
for f in $files; do cmp -s "$tmp/wt/$f" "$sub/$f" || { echo "verification failed for $f" >&2; exit 1; }; done
echo "wrote $out"
