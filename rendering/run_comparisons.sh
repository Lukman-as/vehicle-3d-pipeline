#!/usr/bin/env bash
# Run make_comparisons.py with the interpreter that actually has the deps.
#
# There are two Anaconda installs on this machine and both call themselves
# "(base)".  Only /opt/anaconda3 has cv2 + OpenGL (needed by this script); the
# home ~/anaconda3 base does not, so `python3 make_comparisons.py` fails on
# `import cv2`.  This wrapper always uses the right interpreter.
#
# Usage:
#   ./run_comparisons.sh                  # interactive, 50 biggest cars
#   ./run_comparisons.sh --top 3 --force  # quick 3-car test (redo from scratch)
#   ./run_comparisons.sh <any make_comparisons.py args...>
#
# In each GL window: drag=rotate  arrows=pan  W/S=zoom  ENTER=capture  ESC=skip
set -euo pipefail

PY="/opt/anaconda3/bin/python3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -x "$PY" ]; then
  echo "error: $PY not found." >&2
  echo "Edit PY at the top of this script to an env that has cv2 + OpenGL." >&2
  exit 1
fi

# No args -> the default interactive top-50 run into ../comparisons_top50.
# Any args you pass replace these defaults entirely.
if [ "$#" -eq 0 ]; then
  set -- --top 50 --interactive --out ../comparisons_top50
fi

echo "using $PY"
exec "$PY" make_comparisons.py "$@"
