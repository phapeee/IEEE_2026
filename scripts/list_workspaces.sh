#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# A valid workspace follows the WS_* naming convention and includes:
# - build.sh
# - src/ (directory)

shopt -s nullglob
for dir in "${ROOT_DIR}"/WS_*; do
  [[ -d "${dir}" ]] || continue
  [[ -f "${dir}/build.sh" ]] || continue
  [[ -d "${dir}/src" ]] || continue
  basename "${dir}"
done
