#!/usr/bin/env bash
set -euo pipefail

if ! command -v gpioget >/dev/null 2>&1; then
  echo "gpioget is required. Install it with: sudo apt install gpiod" >&2
  exit 1
fi

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <gpiochip> <line> [line ...]" >&2
  exit 1
fi

chip="$1"
shift

for line in "$@"; do
  gpioget --bias=pull-up "$chip" "$line" >/dev/null
done
