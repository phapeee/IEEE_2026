#!/usr/bin/env bash
set -euo pipefail

# Apply pull-up bias to GPIO lines 18/24/16/1 using gpiomon.
PINS=(18 24 16 1)

if ! command -v gpiomon >/dev/null 2>&1; then
  echo "gpiomon is required. Install it with 'sudo apt install gpiod'." >&2
  exit 1
fi

for pin in "${PINS[@]}"; do
  echo "Requesting pull-up bias on GPIO${pin}"
  gpiomon --silent --bias=pull-up gpiochip0 "$pin" >/dev/null 2>&1 &
  pid=$!
  sleep 0.2
  if kill -0 "$pid" >/dev/null 2>&1; then
    kill "$pid" >/dev/null 2>&1 || true
    wait "$pid" 2>/dev/null || true
  fi
done

echo "Pull-up bias requested for GPIO ${PINS[*]}."
