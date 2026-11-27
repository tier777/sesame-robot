#!/usr/bin/env bash
# Рама v2 целиком: топооптимизация стенки → рама и крышка отсека (с проверками) → картинки и плита → двойник на v2.
#   bash hardware/v2/body/tools/body.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
step() {
  echo "== $1"
  shift
  if ! "$@"; then echo "сломался шаг: $*" >&2; exit 1; fi
}
step "топооптимизация стенки отсека" .venv/bin/python -c "from body.topopt import wall_density; wall_density()"
step "рама и крышка отсека" .venv/bin/python -m body.frame_v2
step "двойник на v2" bash tools/twin.sh v2
step "картинки и плита" .venv/bin/python -m body.render
