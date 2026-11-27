#!/usr/bin/env bash
# Двойник целиком: геометрия → калибровка → позы прошивки → симуляция → отчёт и видео.
#   bash hardware/v2/tools/twin.sh
set -euo pipefail
cd "$(dirname "$0")/.."
if [ ! -x .venv/bin/python ]; then
  echo "окружение: ставлю Python 3.12 и пакеты"
  uv venv -p 3.12 .venv >/dev/null
  uv pip install -p .venv -r requirements.txt >/dev/null
fi
step() {
  echo "== $1"
  shift
  if ! "$@"; then echo "сломался шаг: $*" >&2; exit 1; fi
}
step "геометрия из STEP" .venv/bin/python -m twin.geometry
step "калибровка" .venv/bin/python -m twin.calibrate
step "позы, симуляция, отчёт" .venv/bin/python -m twin.pipeline
